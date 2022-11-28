// simulates reading from file but actually reads from memory

pub struct FileReader<'a> {
    cursor: usize,
    input: &'a [u8],
}

impl<'a> FileReader<'a> {
    pub fn new(input: &'a [u8]) -> Self {
        Self { cursor: 0, input }
    }

    pub fn read(&mut self, into_buf: &mut [u8]) -> bool {
        let to = self.cursor + into_buf.len();
        if to > self.input.len() {
            self.cursor = 0;
            return false;
        } else {
            into_buf.copy_from_slice(&self.input[self.cursor..to]);
            self.cursor += into_buf.len();
            return true;
        }
    }
}
