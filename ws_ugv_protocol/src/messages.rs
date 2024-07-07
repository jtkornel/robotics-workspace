use serde::*;

#[derive(Serialize)]
pub struct SpeedArgs {
    #[serde(rename = "L")] pub l: f32, #[serde(rename = "R")] pub r: f32
}

impl SpeedArgs {
    pub fn tag(&self) -> i16 {
        1
    }
}

#[derive(Serialize)]
pub struct PWMArgs {
    #[serde(rename = "L")] pub l: i16, #[serde(rename = "R")] pub r: i16
}

impl PWMArgs {
    pub fn tag(&self) -> i16 {
        11
    }
}

pub enum CommandMessage {
    Speed(SpeedArgs),
    PWM(PWMArgs),
    BaseFeedbackEnable
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
            Speed(&'a SpeedArgs),
            PWM(&'a PWMArgs),
            BaseFeedbackEnable,
        }

        #[derive(Serialize)]
        struct TypedMessage<'a> {
            #[serde(rename = "T")]
            op: i16,
            #[serde(flatten)]
            msg: Message_<'a>,
        }

        let msg = match self {
            CommandMessage::Speed(t) => TypedMessage { op: t.tag(), msg: Message_::Speed(t) },
            CommandMessage::PWM(t) => TypedMessage { op: t.tag(), msg: Message_::PWM(t) },
            CommandMessage::BaseFeedbackEnable => TypedMessage { op: 3, msg: Message_::BaseFeedbackEnable },
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
    pub gx: f32, pub gy: f32, pub gz: f32,
    pub ax: f32, pub ay: f32, pub az: f32,
    #[serde(rename = "r")]
    pub r_angle: f32,
    #[serde(rename = "p")]
    pub p_angle: f32,
    #[serde(rename = "y")]
    pub y_angle: f32,
    pub q0: f32, pub q1: f32, pub q2: f32, pub q3: f32,
    pub odl: f32, pub odr: f32,
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

#[derive(Debug, PartialEq)]
pub enum FeedbackMessage {
    BaseInfo(BaseInfoData)
}

use serde_json::Value;

impl<'de> serde::Deserialize<'de> for FeedbackMessage {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let value = Value::deserialize(d)?;

        Ok(match value.get("T").and_then(Value::as_i64).unwrap() {
            BaseInfoData::TAG => FeedbackMessage::BaseInfo(BaseInfoData::deserialize(value).unwrap()),
            type_ => panic!("unsupported type {:?}", type_),
        })
    }
}