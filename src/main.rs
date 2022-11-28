#![no_std]
#![no_main]

mod file;
mod stats;
mod time;

// Tested with nRF52840-DK and a UDA1334a DAC
use nrf52840_hal as hal;
use panic_probe as _; // panic magic

#[rtic::app(device = crate::hal::pac, peripherals = true, dispatchers = [SWI0_EGU0, SWI1_EGU1])]
mod app {
    use core::mem;
    use defmt::info;
    use defmt_rtt as _; // global logger;
    use embedded_dma::ReadTarget;
    use hal::{gpio::Level, i2s::*, pac::interrupt, rtc::RtcInterrupt, Rtc};
    use heapless::{
        pool,
        pool::singleton::{Box, Pool},
        spsc::{Consumer, Producer, Queue},
    };
    use lc3_codec::{
        common::{
            complex::Complex,
            config::{FrameDuration, SamplingFrequency},
        },
        decoder::lc3_decoder::Lc3Decoder,
    };
    use nrf52840_hal as hal;

    // The I2S buffer address must be 4 byte aligned.
    static mut PLAY_BUF0: Aligned<[i16; DMA_LEN]> = Aligned([0i16; DMA_LEN]);
    static mut PLAY_BUF1: Aligned<[i16; DMA_LEN]> = Aligned([0i16; DMA_LEN]);

    const INPUT_FILE: &'static [u8] = include_bytes!("../48khz_16bit_mono_10ms_150byte_piano.lc3");
    const NUM_CH: usize = 1;
    const FILE_FRAME_LEN: usize = 150;
    const DMA_LEN: usize = 960;
    const INPUT_QUEUE_LEN: usize = 5; // TODO: shouldn't this be 6 because of N-1?

    #[repr(align(4))]
    pub struct Aligned<T: ?Sized>(T);

    unsafe impl ReadTarget for Aligned<[i16; DMA_LEN]> {
        type Word = i16;

        fn as_read_buffer(&self) -> (*const Self::Word, usize) {
            let len = mem::size_of_val(&self.0) / mem::size_of::<Self::Word>();
            let ptr = &self.0 as *const _ as *const Self::Word;
            (ptr, len)
        }
    }

    type AudioFrame = [i16; DMA_LEN];
    type FileFrame = [u8; FILE_FRAME_LEN];
    pool!(P: AudioFrame);
    pool!(B: FileFrame);

    // Resources shared between tasks
    #[shared]
    struct Shared {}

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        transfer: Option<Transfer<&'static Aligned<[i16; DMA_LEN]>>>,
        decoder: Lc3Decoder<'static, NUM_CH>,
        input_tx: Producer<'static, Box<P>, INPUT_QUEUE_LEN>,
        input_rx: Consumer<'static, Box<P>, INPUT_QUEUE_LEN>,
        bytes: Box<B>,
        samples: Box<P>,
        is_buf0_playing: bool,
    }

    #[init(local = [queue: Queue<bool, 2> = Queue::new(), input_queue: Queue<Box<P>, INPUT_QUEUE_LEN> = Queue::new(), memory_samples: [u8; 11520] = [0; 11520], memory_bytes: [u8; 1200] = [0; 1200]])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        info!("init");

        // Increase the capacity of the memory pool
        P::grow(ctx.local.memory_samples);
        B::grow(ctx.local.memory_bytes);

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
        const SCALER_COMPLEX_LENS: (usize, usize) =
            Lc3Decoder::<NUM_CH>::calc_working_buffer_lengths(DURATION, FREQ);
        static mut SCALER_BUF: [f32; SCALER_COMPLEX_LENS.0] = [0.0; SCALER_COMPLEX_LENS.0];
        static mut COMPLEX_BUF: [Complex; SCALER_COMPLEX_LENS.1] =
            [Complex { r: 0., i: 0. }; SCALER_COMPLEX_LENS.1];
        let decoder =
            Lc3Decoder::<NUM_CH>::new(DURATION, FREQ, unsafe { &mut SCALER_BUF }, unsafe {
                &mut COMPLEX_BUF
            });

        let p0 = hal::gpio::p0::Parts::new(p.P0);

        // Configure I2S controller
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
        i2s.set_mck_frequency(MckFreq::_32MDiv21);
        i2s.set_ratio(Ratio::_32x);
        i2s.enable_interrupt(I2SEvent::TxPtrUpdated).start();

        // send an empty transfer to kick things off
        let transfer = i2s.tx(unsafe { &PLAY_BUF0 }).ok();

        let (input_tx, input_rx) = ctx.local.input_queue.split();
        let bytes = B::alloc().unwrap().init([0; FILE_FRAME_LEN]);
        let samples = P::alloc().unwrap().init([0; DMA_LEN]);
        info!("done with init");

        (
            Shared {},
            Local {
                transfer,
                decoder,
                input_tx,
                input_rx,
                bytes,
                samples,
                is_buf0_playing: true,
            },
            init::Monotonics(),
        )
    }

    #[idle(local=[input_tx, decoder, bytes, samples])]
    fn idle(ctx: idle::Context) -> ! {
        info!("idle");
        let input_tx = ctx.local.input_tx;
        let decoder = ctx.local.decoder;
        let bytes = ctx.local.bytes;
        let samples = ctx.local.samples;

        // simulates reading a file from disk
        let mut file_reader = crate::file::FileReader::new(INPUT_FILE);

        const SAMPLES_OUT_LEN: usize = DMA_LEN / 2;
        let dec_in_buffer = bytes.as_mut_slice();
        let mut samples_out_left = &mut samples[..SAMPLES_OUT_LEN];
        let mut stats = crate::stats::Stats::new();

        loop {
            if input_tx.ready() {
                stats.start_frame();
                if !file_reader.read(dec_in_buffer) {
                    // start reading the file again
                    info!("start reading the file again");
                    stats = crate::stats::Stats::new();
                    continue;
                }

                // decode a single channel
                decoder
                    .decode_frame(16, 0, &dec_in_buffer, &mut samples_out_left)
                    .unwrap();

                let mut dec_out_buffer = P::alloc().unwrap().init([0; DMA_LEN]);

                // interleve the samples (l-r-l-r-l-r etc.)
                for (index, l) in samples_out_left.iter().enumerate() {
                    dec_out_buffer[index * 2] = *l;
                    dec_out_buffer[index * 2 + 1] = *l; // left and right channels get the same data (i.e mono)
                }

                // since we have already checked if there is space and
                // this is the only function that can add to the queue this is safe to add
                input_tx.enqueue(dec_out_buffer).unwrap();

                stats.calc_and_print_uptime();
            } else {
                //  Input queue full
                cortex_m::asm::wfi();
            }
        }
    }

    #[task(binds = I2S, local = [transfer, input_rx, is_buf0_playing])]
    fn i2s_transfer(ctx: i2s_transfer::Context) {
        let transfer = ctx.local.transfer;
        let input_rx = ctx.local.input_rx;
        let is_buf0_playing = ctx.local.is_buf0_playing;
        let (_, i2s) = transfer.take().unwrap().wait();

        if i2s.is_event_triggered(I2SEvent::TxPtrUpdated) {
            *is_buf0_playing = !*is_buf0_playing;

            // use a double buffer
            // safety: no other function touches these buffers besides the DMA transfer after `init`
            //         and this function is not reentrant (I believe)
            let (alligned_buf, buf) = if *is_buf0_playing {
                unsafe { (&PLAY_BUF0, &mut PLAY_BUF0.0) }
            } else {
                unsafe { (&PLAY_BUF1, &mut PLAY_BUF1.0) }
            };

            match input_rx.dequeue() {
                Some(buf_in) => {
                    buf.copy_from_slice(buf_in.as_slice());
                }
                None => {
                    // create silence here
                    for x in buf {
                        *x = 0;
                    }

                    info!("no input available")
                }
            }

            i2s.reset_event(I2SEvent::TxPtrUpdated);
            *transfer = i2s.tx(alligned_buf).ok();
        }
    }

    #[interrupt]
    fn RTC0() {
        crate::time::process_tick();
    }
}
