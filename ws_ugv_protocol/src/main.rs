use std::io::BufReader;

use ws_ugv_protocol::messages::*;
use ws_ugv_protocol::*;

use serial2::SerialPort;

fn main() {
    let readport = SerialPort::open("/dev/serial0", 115200).unwrap();
    let mut reader = BufReader::new(readport);

    let mut writeport = SerialPort::open("/dev/serial0", 115200).unwrap();
    writeport.set_rts(false).unwrap();
    writeport.set_dtr(false).unwrap();

    let flow_off = CommandMessage::SetBaseFeedbackFlow(BaseFeedbackFlowArgs {cmd: 0});
    let _ = write_command(& mut writeport, flow_off);

    let _ = write_command(& mut writeport, CommandMessage::GetBaseFeedback);
    let msg = read_feedback(& mut reader).unwrap();
    println!("Received message: {:?}", msg);

    let _ = write_command(& mut writeport, CommandMessage::GetIMUData);
    let msg = read_feedback(& mut reader).unwrap();
    println!("Received message: {:?}", msg);
}
