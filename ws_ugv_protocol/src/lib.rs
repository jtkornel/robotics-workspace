use std::io::{Write, BufRead};
use std::io;

pub mod messages;

use crate::messages::{CommandMessage, FeedbackMessage};

// Read a feedback message from the UGV
pub fn read_feedback<R: BufRead>(reader: & mut R) -> Result<FeedbackMessage, io::Error>
{
    let mut buf = String::new();
    let num_bytes = reader.read_line(&mut buf)?;

    if num_bytes == 0
    {
        return Err(io::Error::other("No data"));
    }

    let deserialized : FeedbackMessage = serde_json::from_str(&buf)?;

    Ok(deserialized)
}

// Write a command message to the UGV
// Returns the number of bytes written
pub fn write_command<W: Write>(writer: & mut W, msg: CommandMessage)-> Result<usize, io::Error>
{
    let json_msg = serde_json::to_string(&msg)?;
    let num_bytes = writer.write(json_msg.as_bytes())?;
    Ok(num_bytes)
}


#[cfg(test)]
mod tests {

    use super::*;
    use std::io::{Cursor, BufReader};
    use crate::messages::{FeedbackMessage, BaseInfoData, CommandMessage, SpeedArgs};

    // Use cursors to simulate the serial port / network connection
    fn create_fake_reader(rx_message: &str) ->  BufReader<Cursor<&[u8]>> {
        BufReader::new(Cursor::new(rx_message.as_bytes()))
    }

    fn create_fake_writer() -> Cursor<Vec<u8>> {
        let tx_message: Vec<u8> = Vec::new();
        Cursor::new(tx_message)
    }

    #[test]
    fn test_read_feedback() {
        let rx_object = FeedbackMessage::BaseInfo(BaseInfoData{ l: 0.2, r: 0.6, gx: 0.3, gy: 0.4, gz: 0.0, ax: 0.0, ay: 0.0, az: 0.0, r_angle: 0.0, p_angle: 0.0, y_angle: 0.0, q0: 1.0, q1: 0.0, q2: 0.0, q3: 0.0, odl: 0.0, odr: 0.0, v: 11.0, a_b: None, a_s: None, a_e: None, a_t: None, tor_b: None, tor_s: None, tor_e: None, tor_h: None, pan: None, tilt: None});
        let rx_message = r#"{"T":1001,"L":0.2,"R":0.6,"gx":0.3,"gy":0.4,"gz":0,"ax":0,"ay":0,"az":0,"r":0,"p":0,"y":0,"q0":1.0, "q1":0, "q2":0, "q3":0,"odl":0,"odr":0,"v":11.0}"#;

        let mut comm = create_fake_reader(rx_message);

        let deserialized  = read_feedback(&mut comm);

        assert!(deserialized.is_ok());
        assert_eq!(deserialized.unwrap(), rx_object);
    }

    #[test]
    fn test_read_feedback_empty() {
        let mut comm = create_fake_writer();

        let deserialized  = read_feedback(&mut comm);

        assert!(deserialized.is_err());
    }

    #[test]
    fn test_write_command() {
        let tx_object = CommandMessage::Speed(SpeedArgs {l: 0.5, r: 0.5});

        let mut writer = create_fake_writer();

        // Nothing written initially
        assert_eq!(writer.get_ref().len(), 0);

        let res = write_command(&mut writer, tx_object);

        // Check that something was written, with the expected length
        assert!(res.is_ok());
        let bytes_written = res.unwrap();
        assert!(bytes_written > 0);
        assert_eq!(writer.get_ref().len(), bytes_written);

        // Check that the written message is valid JSON
        use serde_json::Value;
        let value: Result<Value, serde_json::Error> = serde_json::from_str(std::str::from_utf8(writer.get_ref()).unwrap());

        assert!(value.is_ok());
    }


}