use serde::*;

#[derive(Serialize)]
pub struct SpeedArgs {
    // Positive values are forward, negative values are reverse
    #[serde(rename = "L")] pub l: f32, // Left wheel speed, range [-0.5, 0.5]
    #[serde(rename = "R")] pub r: f32  // Right wheel speed, range [-0.5, 0.5]
}

impl SpeedArgs {
    pub fn tag(&self) -> i16 {
        1
    }
}

#[derive(Serialize)]
pub struct PWMArgs {
    // Positive values are forward, negative values are reverse
    #[serde(rename = "L")] pub l: i16, // Left motor PWM, range [-255, 255]
    #[serde(rename = "R")] pub r: i16  // Right motor PWM, range [-255, 255]
}

impl PWMArgs {
    pub fn tag(&self) -> i16 {
        11
    }
}

#[derive(Serialize)]
pub struct RosCtrlArgs {
    #[serde(rename = "X")] pub x: f32, // Linear velocity in ms/s
    #[serde(rename = "Z")] pub z: f32  // Angular velocity in rad/s
}

impl RosCtrlArgs {
    pub fn tag(&self) -> i16 {
        13
    }
}

#[derive(Serialize)]
pub struct MotorPIDArgs {
    #[serde(rename = "P")] pub p: f32, // Proportional gain
    #[serde(rename = "I")] pub i: f32, // Integral gain
    #[serde(rename = "D")] pub d: f32, // Derivative gain
    #[serde(rename = "L")] pub l: f32, // Windup limit (reserved for future use)
}

impl MotorPIDArgs {
    pub fn tag(&self) -> i16 {
        2
    }
}

#[derive(Serialize)]
pub struct OLEDScreenControlArgs {
    #[serde(rename = "Text")] pub text: String
}

impl OLEDScreenControlArgs {
    pub fn tag(&self) -> i16 {
        3
    }
}

#[derive(Serialize)]
pub struct BaseFeedbackFlowArgs {
    pub cmd: i16  // 1 to enable, 0 to disable
}

impl BaseFeedbackFlowArgs {
    pub fn tag(&self) -> i16 {
        131
    }
}

#[derive(Serialize)]
pub struct IMUOffsetArgs {
    #[serde(rename = "gx")] pub gx: f32,
    #[serde(rename = "gy")] pub gy: f32,
    #[serde(rename = "gz")] pub gz: f32,
    #[serde(rename = "ax")] pub ax: f32,
    #[serde(rename = "ay")] pub ay: f32,
    #[serde(rename = "az")] pub az: f32,
    #[serde(rename = "cx")] pub cx: f32,
    #[serde(rename = "cy")] pub cy: f32,
    #[serde(rename = "cz")] pub cz: f32,
}

impl IMUOffsetArgs {
    pub fn tag(&self) -> i16 {
        129
    }
}

pub enum CommandMessage {
    EmergencyStop,
    Speed(SpeedArgs),
    PWM(PWMArgs),
    RosCtrl(RosCtrlArgs),   // Only for UGV01 with encoder
    MotorPID(MotorPIDArgs), // Only for UGV01 with encoder
    OLEDScreenControl(OLEDScreenControlArgs),
    OLEDScreenRestore,
    GetIMUData,
    CalibrateIMU,
    GetIMUOffset,
    SetIMUOffset(IMUOffsetArgs),
    GetBaseFeedback,
    SetBaseFeedbackFlow(BaseFeedbackFlowArgs)
}

// Workaround for serde not supporting enums with integer tags
// https://stackoverflow.com/questions/65575385/deserialization-of-json-with-serde-by-a-numerical-value-as-type-identifier/65576570#65576570
impl Serialize for CommandMessage {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {

        #[derive(Serialize)]
        #[serde(untagged)]
        enum Message_<'a> {
            EmergencyStop,
            Speed(&'a SpeedArgs),
            PWM(&'a PWMArgs),
            RosCtrl(&'a RosCtrlArgs),
            MotorPID(&'a MotorPIDArgs),
            OLEDScreenControl(&'a OLEDScreenControlArgs),
            OLEDScreenRestore,
            GetIMUData,
            CalibrateIMU,
            GetIMUOffset,
            SetIMUOffset(&'a IMUOffsetArgs),
            GetBaseFeedback,
            SetBaseFeedbackFlow(&'a BaseFeedbackFlowArgs)
        }

        #[derive(Serialize)]
        struct TypedMessage<'a> {
            #[serde(rename = "T")]
            op: i16,
            #[serde(flatten)]
            msg: Message_<'a>,
        }

        let msg = match self {
            CommandMessage::EmergencyStop => TypedMessage { op: 0, msg: Message_::EmergencyStop },
            CommandMessage::Speed(t) => TypedMessage { op: t.tag(), msg: Message_::Speed(t) },
            CommandMessage::PWM(t) => TypedMessage { op: t.tag(), msg: Message_::PWM(t) },
            CommandMessage::RosCtrl(t) => TypedMessage { op: t.tag(), msg: Message_::RosCtrl(t) },
            CommandMessage::MotorPID(t) => TypedMessage { op: t.tag(), msg: Message_::MotorPID(t) },
            CommandMessage::OLEDScreenControl(t) => TypedMessage { op: t.tag(), msg: Message_::OLEDScreenControl(t) },
            CommandMessage::OLEDScreenRestore => TypedMessage { op: -3, msg: Message_::OLEDScreenRestore },
            CommandMessage::GetIMUData => TypedMessage { op: 126, msg: Message_::GetIMUData },
            CommandMessage::CalibrateIMU => TypedMessage { op: 127, msg: Message_::CalibrateIMU },
            CommandMessage::GetIMUOffset => TypedMessage { op: 128, msg: Message_::GetIMUOffset },
            CommandMessage::SetIMUOffset(t) => TypedMessage { op: t.tag(), msg: Message_::SetIMUOffset(t) },
            CommandMessage::GetBaseFeedback => TypedMessage { op: 130, msg: Message_::GetBaseFeedback },
            CommandMessage::SetBaseFeedbackFlow(t) => TypedMessage { op: t.tag(), msg: Message_::SetBaseFeedbackFlow(t) }
        };
        msg.serialize(serializer)
    }
}

#[derive(Deserialize, Debug, PartialEq)]
pub struct BaseInfoData {
    // Renamings to wire format accepted by UGV
    #[serde(rename = "L")]
    pub l: f32,
    #[serde(rename = "R")]
    pub r: f32,
    pub gx: Option<f32>, pub gy: Option<f32>, pub gz: Option<f32>,
    pub ax: Option<f32>, pub ay: Option<f32>, pub az: Option<f32>,
    #[serde(rename = "r")]
    pub r_angle: f32,
    #[serde(rename = "p")]
    pub p_angle: f32,
    #[serde(rename = "y")]
    pub y_angle: Option<f32>,
    pub q0: Option<f32>, pub q1: Option<f32>, pub q2: Option<f32>, pub q3: Option<f32>,
    pub odl: Option<f32>, pub odr: Option<f32>,
    pub v: f32,
    #[serde(rename = "ab")]
    pub a_b: Option<f32>,
    #[serde(rename = "as")]
    pub a_s: Option<f32>,
    #[serde(rename = "ae")]
    pub a_e: Option<f32>,
    #[serde(rename = "at")]
    pub a_t: Option<f32>,
    #[serde(rename = "torB")]
    pub tor_b: Option<f32>,
    #[serde(rename = "torS")]
    pub tor_s: Option<f32>,
    #[serde(rename = "torE")]
    pub tor_e: Option<f32>,
    #[serde(rename = "torH")]
    pub tor_h: Option<f32>,
    pub pan: Option<f32>, pub tilt: Option<f32>
}

impl BaseInfoData {
    const TAG: i64 = 1001;
}

#[derive(Deserialize, Debug, PartialEq)]
pub struct IMUData {
    pub gx: f32, pub gy: f32, pub gz: f32,
    pub ax: f32, pub ay: f32, pub az: f32,
    pub mx: f32, pub my: f32, pub mz: f32
}

impl IMUData {
    const TAG: i64 = 1002;
}

#[derive(Deserialize, Debug, PartialEq)]
pub struct IMUOffsetData {
    pub gx: f32, pub gy: f32, pub gz: f32,
    pub ax: f32, pub ay: f32, pub az: f32,
    pub cx: f32, pub cy: f32, pub cz: f32
}

impl IMUOffsetData {
    const TAG: i64 = 129;
}

#[derive(Debug, PartialEq)]
pub enum FeedbackMessage {
    BaseInfo(BaseInfoData),
    IMU(IMUData),
    IMUOffset(IMUOffsetData)
}

use serde_json::Value;

impl<'de> serde::Deserialize<'de> for FeedbackMessage {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let value = Value::deserialize(d)?;

        Ok(match value.get("T").and_then(Value::as_i64).unwrap() {
            BaseInfoData::TAG => FeedbackMessage::BaseInfo(BaseInfoData::deserialize(value).unwrap()),
            IMUData::TAG => FeedbackMessage::IMU(IMUData::deserialize(value).unwrap()),
            IMUOffsetData::TAG => FeedbackMessage::IMUOffset(IMUOffsetData::deserialize(value).unwrap()),
            type_ => panic!("unsupported type {:?}", type_),
        })
    }
}