#![no_std]
#![no_main]

mod file;
mod stats;
mod time;

// Tested with nRF52840-DK and a UDA1334a DAC
use defmt_rtt as _;
use nrf52840_hal as hal;
use panic_probe as _; // panic magic // global logger

#[rtic::app(device = crate::hal::pac, peripherals = true, dispatchers = [SWI0_EGU0, SWI1_EGU1])]
mod app {
    use crate::stats::Stats;
    use bbqueue::{BBBuffer, SplitGrantR};
    use byteorder::{ByteOrder, LittleEndian};
    use core::mem;
    use defmt::info;
    use defmt_rtt as _; // global logger;
    use embedded_dma::ReadBuffer;
    use hal::{gpio::Level, i2s::*, pac::interrupt, rtc::RtcInterrupt, Rtc};
    use lc3_codec::{
        common::{
            complex::Complex,
            config::{FrameDuration, SamplingFrequency},
        },
        decoder::lc3_decoder::Lc3Decoder,
    };
    use nrf52840_hal as hal;

    static mut SILENCE_BUF: SilenceAudioFrame = SilenceAudioFrame([0u8; AUDIO_FRAME_BYTES_LEN]);
    static BB: BBAudioBuffer = BBAudioBuffer(BBBuffer::new());

    const INPUT_FILE: &'static [u8] = include_bytes!("../48khz_16bit_mono_10ms_150byte_piano.lc3");
    // const INPUT_FILE: &'static [u8] = include_bytes!("../48khz_16bit_mono_10ms_150byte_sine.lc3");
    const NUM_CH: usize = 1;
    const FILE_FRAME_LEN: usize = 150;
    const DMA_LEN: usize = 960;
    const AUDIO_FRAME_BYTES_LEN: usize = DMA_LEN * mem::size_of::<i16>();
    const BB_BYTES_LEN: usize = AUDIO_FRAME_BYTES_LEN * 6;

    // assuming that the BBQueue buffer begins on a 4 byte alligned address and chunk sizes are a multiple of 4 we should have the aligned buffers we need
    // buffers used for I2S DMA transfers must be 4 byte aligned.
    #[derive(Debug)]
    #[repr(align(4))]
    struct BBAudioBuffer(BBBuffer<BB_BYTES_LEN>);

    // the I2S buffer address must be 4 byte aligned.
    #[derive(Debug)]
    #[repr(align(4))]
    pub struct SilenceAudioFrame([u8; AUDIO_FRAME_BYTES_LEN]);

    pub struct ReadGrantAudioFrame {
        inner: Option<SplitGrantR<'static, BB_BYTES_LEN>>,
    }

    unsafe impl ReadBuffer for ReadGrantAudioFrame {
        type Word = i16;

        unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
            let buf = match &self.inner {
                Some(rgr) => {
                    // if circular buffer has wrapped around then buf0 will be zero length and we take buf1 instead
                    let (buf0, buf1) = rgr.bufs();
                    if buf0.len() >= AUDIO_FRAME_BYTES_LEN {
                        &buf0[..AUDIO_FRAME_BYTES_LEN]
                    } else {
                        &buf1[..AUDIO_FRAME_BYTES_LEN]
                    }
                }
                None => &SILENCE_BUF.0,
            };

            // interpret the bytes in AudioFrame as i16 integers
            let len = mem::size_of_val(buf) / mem::size_of::<Self::Word>();
            let ptr = buf as *const _ as *const Self::Word;
            (ptr, len)
        }
    }

    // resources shared between tasks
    #[shared]
    struct Shared {}

    // local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        transfer: Option<Transfer<ReadGrantAudioFrame>>,
        decoder: Lc3Decoder<'static, NUM_CH>,
        producer: bbqueue::Producer<'static, BB_BYTES_LEN>,
        consumer: bbqueue::Consumer<'static, BB_BYTES_LEN>,
    }

    #[init(local = [])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        info!("init");

        let p = ctx.device;
        let _clocks = hal::clocks::Clocks::new(p.CLOCK)
            .enable_ext_hfosc()
            .start_lfclk(); // required for uptime()

        // this is required to measure uptime
        let mut rtc = Rtc::new(p.RTC0, 0).unwrap();
        rtc.enable_interrupt(RtcInterrupt::Overflow, None);
        rtc.enable_counter();

        // configure the LC3 decoder
        const FREQ: SamplingFrequency = SamplingFrequency::Hz48000;
        const DURATION: FrameDuration = FrameDuration::TenMs;
        // TODO: upgrade to use heapless pool for buffers (this is horrible)
        const SCALER_COMPLEX_LENS: (usize, usize) =
            Lc3Decoder::<NUM_CH>::calc_working_buffer_lengths(DURATION, FREQ);
        static mut SCALER_BUF: [f32; SCALER_COMPLEX_LENS.0] = [0.0; SCALER_COMPLEX_LENS.0];
        static mut COMPLEX_BUF: [Complex; SCALER_COMPLEX_LENS.1] =
            [Complex { r: 0., i: 0. }; SCALER_COMPLEX_LENS.1];
        let decoder =
            Lc3Decoder::<NUM_CH>::new(DURATION, FREQ, unsafe { &mut SCALER_BUF }, unsafe {
                &mut COMPLEX_BUF
            });

        // configure I2S controller
        let p0 = hal::gpio::p0::Parts::new(p.P0);
        let sck_green = p0.p0_29.into_push_pull_output(Level::Low).degrade(); // bclk
        let sdout_yellow = Some(p0.p0_30.into_push_pull_output(Level::Low).degrade()); // din
        let lrck_orange = p0.p0_31.into_push_pull_output(Level::Low).degrade(); // wsel

        let pins = hal::i2s::Pins::Controller {
            lrck: lrck_orange,
            mck: None,
            sck: sck_green,
            sdout: sdout_yellow,
            sdin: None,
        };
        let i2s = I2S::new(p.I2S, pins);
        i2s.set_channels(Channels::Stereo);
        i2s.set_sample_width(SampleWidth::_16bit);
        unsafe { i2s.set_buffersize(DMA_LEN as u32).ok() };
        i2s.set_format(Format::I2S);
        i2s.set_mck_frequency(MckFreq::_32MDiv21); // closest approximation to 48Khz you can get
        i2s.set_ratio(Ratio::_32x);
        i2s.enable_interrupt(I2SEvent::TxPtrUpdated);
        i2s.start();

        let (producer, consumer) = BB.0.try_split().unwrap();
        let read_grant = ReadGrantAudioFrame { inner: None };
        let transfer = i2s.tx(read_grant).ok();
        info!("done with init");

        (
            Shared {},
            Local {
                transfer,
                decoder,
                producer,
                consumer,
            },
            init::Monotonics(),
        )
    }

    #[idle(local=[decoder, producer])]
    fn idle(ctx: idle::Context) -> ! {
        info!("idle");
        let decoder = ctx.local.decoder;
        let producer = ctx.local.producer;

        // simulates reading a file from disk
        let mut file_reader = crate::file::FileReader::new(INPUT_FILE);

        const SAMPLES_OUT_LEN: usize = DMA_LEN / 2;
        let mut stats = Stats::new();

        loop {
            match producer.grant_exact(AUDIO_FRAME_BYTES_LEN) {
                Ok(mut wgr) => {
                    stats.start_frame();
                    let mut dec_in_buffer = [0; FILE_FRAME_LEN];

                    if !file_reader.read(&mut dec_in_buffer) {
                        // start reading the file again
                        info!("start reading the file again");
                        stats = crate::stats::Stats::new();
                        continue;
                    }

                    let mut samples_out_left = [0; SAMPLES_OUT_LEN];

                    // decode a single channel
                    decoder
                        .decode_frame(16, 0, &dec_in_buffer, &mut samples_out_left)
                        .unwrap();

                    let mut dec_out_buffer = [0i16; DMA_LEN];

                    // interleve the samples (l-r-l-r-l-r etc.)
                    for (index, l) in samples_out_left.iter().enumerate() {
                        dec_out_buffer[index * 2] = *l;
                        dec_out_buffer[index * 2 + 1] = *l; // left and right channels get the same data (i.e mono)
                    }

                    wgr.to_commit(AUDIO_FRAME_BYTES_LEN);
                    LittleEndian::write_i16_into(&dec_out_buffer, wgr.buf());
                    stats.calc_and_print_uptime();
                }
                Err(_) => {
                    // input queue full, this is normal
                    // the i2s interrupt firing should free up space
                    cortex_m::asm::wfi();
                }
            }
        }
    }

    // the root cause of the problem is that the code in this function is too slow
    #[task(binds = I2S, local = [transfer, consumer])]
    fn i2s_transfer(ctx: i2s_transfer::Context) {
        let transfer = ctx.local.transfer;
        let consumer = ctx.local.consumer;
        let (prev_play_buf, i2s) = transfer.take().expect("nothing transfered").wait();

        // i2s is done playing the previous buffer, time to reclaim the memory
        match prev_play_buf.inner {
            Some(rgr) => rgr.release(AUDIO_FRAME_BYTES_LEN),
            None => {}
        }

        // lock free read
        let read_grant = match consumer.split_read() {
            Ok(rgr) => ReadGrantAudioFrame { inner: Some(rgr) },
            Err(_) => {
                // decoding cannot keep up with playback speed - play silence instead
                info!("silence");
                ReadGrantAudioFrame { inner: None }
            }
        };

        i2s.reset_event(I2SEvent::TxPtrUpdated);
        *transfer = i2s.tx(read_grant).ok();
    }

    #[interrupt]
    fn RTC0() {
        crate::time::process_tick();
    }
}
