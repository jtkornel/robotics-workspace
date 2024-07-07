use std::io::BufReader;


use ws_ugv_protocol::messages::*;
use ws_ugv_protocol::*;

use std::io::Cursor;

fn main() {
    let rx_message = r#"{"T":1001,"L":0,"R":0,"gx":0,"gy":0,"gz":0,"ax":0,"ay":0,"az":0,"r":0,"p":0,"y":0,"q0":0, "q1":0, "q2":0, "q3":0,"odl":0,"odr":0,"v":11.0}"#;
    let mut tx_message: Vec<u8> = Vec::new();

    let rx_buff = BufReader::new(Cursor::new(rx_message));
    let tx_buff = Cursor::new(tx_message);

    let mut comm = UGVComm{ command_writer: tx_buff, feedback_reader: rx_buff};

    let deserialized : FeedbackMessage = comm.read_feedback().unwrap();

    println!("{:?}", deserialized);
}
