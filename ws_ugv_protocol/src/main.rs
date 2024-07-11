use std::io::BufReader;


use ws_ugv_protocol::messages::*;
use ws_ugv_protocol::*;

use std::io::Cursor;

fn main() {

    let port = serialport::new("/dev/ttyUSB0", 9600).open().unwrap();

    //let mut comm = UGVComm{ command_writer: &port, feedback_reader: BufReader::new(port) };

    //let deserialized : FeedbackMessage = comm.read_feedback().unwrap();

    //println!("{:?}", deserialized);
}
