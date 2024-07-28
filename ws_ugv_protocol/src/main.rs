
use ws_ugv_protocol::messages::*;
use ws_ugv_protocol::*;

use tokio_serial::{SerialPort, SerialPortBuilderExt, SerialStream};
use tokio::io::BufReader;

fn main() {
    let rt = tokio::runtime::Runtime::new().unwrap();
    let future = app();
    rt.block_on(future);
}

async fn write_all_commands(mut writeport: SerialStream) {
    let _ = write_command(& mut writeport, CommandMessage::GetBaseFeedback);
    let _ = write_command(& mut writeport, CommandMessage::GetIMUData);
}

async fn read_all_feedback(mut readport: BufReader<SerialStream>) -> (FeedbackMessage, FeedbackMessage) {
    let base_feedback = read_feedback(& mut readport).await.unwrap();
    let imu_feedback = read_feedback(& mut readport).await.unwrap();

    (base_feedback, imu_feedback)
}

async fn app() {
    let readport = tokio_serial::new("/dev/serial0", 115200).open_native_async().expect("Failed to open port");
    let breader = BufReader::new(readport);

    let mut writeport = tokio_serial::new("/dev/serial0", 115200).open_native_async().expect("Failed to open port");
    writeport.write_request_to_send(false).unwrap();
    writeport.write_data_terminal_ready(false).unwrap();

    let flow_off = CommandMessage::SetBaseFeedbackFlow(BaseFeedbackFlowArgs {cmd: 0});
    let _ = write_command(& mut writeport, flow_off);

    let wr = write_all_commands(writeport);
    let rd = read_all_feedback(breader);
    wr.await;
    rd.await;
}
