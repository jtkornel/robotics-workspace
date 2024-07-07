use std::io::{Write, BufRead};
use std::io;

pub mod messages;

use crate::messages::{CommandMessage, FeedbackMessage};

// The UGV communication object
pub struct UGVComm<W, R>
    where
    W: Write,
    R: BufRead
{
    pub command_writer: W,
    pub feedback_reader: R
}


impl<W: Write, R: BufRead> UGVComm<W,R> {

// Read a feedback message from the UGV
pub fn read_feedback(&mut self) -> Result<FeedbackMessage, io::Error>
{
    let mut buf = String::new();
    let num_bytes = self.feedback_reader.read_line(&mut buf)?;

    if num_bytes == 0
    {
        return Err(io::Error::other("No data"));
    }

    let deserialized : FeedbackMessage = serde_json::from_str(&buf)?;

    Ok(deserialized)
}

// Write a command message to the UGV
pub fn write_command(&mut self, msg: CommandMessage)-> Result<usize, io::Error>
{
    let json_msg = serde_json::to_string(&msg)?;
    let num_bytes = self.command_writer.write(json_msg.as_bytes())?;
    Ok(num_bytes)
}

}