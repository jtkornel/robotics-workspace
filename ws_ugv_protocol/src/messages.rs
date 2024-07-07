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

#[derive(Deserialize, Debug)]
pub struct BaseInfoData {
    // Renamings to wire format accepted by UGV
    #[serde(rename = "L")]
    l: f32,
    #[serde(rename = "R")]
    r: f32,
    gx: f32, gy: f32, gz: f32,
    ax: f32, ay: f32, az: f32,
    #[serde(rename = "r")]
    r_angle: f32,
    #[serde(rename = "p")]
    p_angle: f32,
    #[serde(rename = "y")]
    y_angle: f32,
    q0: f32, q1: f32, q2: f32, q3: f32,
    odl: f32, odr: f32,
    v: f32,
    #[serde(rename = "ab")]
    a_b: Option<f32>,
    #[serde(rename = "as")]
    a_s: Option<f32>,
    #[serde(rename = "ae")]
    a_e: Option<f32>,
    #[serde(rename = "at")]
    a_t: Option<f32>,
    #[serde(rename = "torB")]
    tor_b: Option<f32>,
    #[serde(rename = "torS")]
    tor_s: Option<f32>,
    #[serde(rename = "torE")]
    tor_e: Option<f32>,
    #[serde(rename = "torH")]
    tor_h: Option<f32>,
    pan: Option<f32>, tilt: Option<f32>
}

impl BaseInfoData {
    const TAG: i64 = 1001;
}

#[derive(Debug)]
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