//! Tests of the `cam.features` and `cam.suggest` contracts: their keys, and replies that match
//! their output schemas.

use serde_json::{Value, json};

use super::{CamFeaturesParams, CamFeaturesReply, CamSuggestParams, CamSuggestReply};
use crate::Session;
use crate::schema::tests::{assert_keys, conforms};
use crate::schema::{ParamCheck, input_schema_for, output_schema_for};

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    s.execute(id, &p).unwrap_or_else(|e| panic!("{id} {p}: {e}"))
}

#[test]
fn schemas_and_the_key_check_agree() {
    assert_keys(input_schema_for::<CamFeaturesParams>(), &["axis", "bodies", "origin", "picks", "tolerance"]);
    assert_keys(
        input_schema_for::<CamSuggestParams>(),
        &["as_picks", "axis", "bodies", "check", "origin", "post", "profile_path", "stock_top", "tolerance", "tool_library_path", "tools"],
    );
}

/// Both commands reply as their output schemas say, in Warn without warnings; strict refuses a
/// misspelt key.
#[test]
fn replies_match_the_output_schemas() {
    let matches = |schema: schemars::Schema, r: &Value| {
        let schema = schema.as_value();
        assert!(conforms(schema, schema, r), "{r} doesn't match {schema}");
        assert!(r.get("param_warnings").is_none(), "{r}");
    };
    let mut s = Session { param_check: ParamCheck::Warn, ..Session::default() };
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 10, "corner": [-20, -15, -10]}));
    run(&mut s, "solid.cylinder", json!({"radius": 3, "height": 12, "base": [0, 0, -11], "operation": "cut"}));
    matches(output_schema_for::<CamFeaturesReply>(), &run(&mut s, "cam.features", json!({})));
    matches(output_schema_for::<CamFeaturesReply>(), &run(&mut s, "cam.features", json!({"axis": "a"})));
    matches(output_schema_for::<CamFeaturesReply>(), &run(&mut s, "cam.features", json!({"picks": true})));
    let tools =
        json!([{"number": 1, "kind": "flat_endmill", "diameter": 4, "flute_length": 15, "spindle_rpm": 12000, "feed": 800, "plunge_feed": 200}]);
    matches(output_schema_for::<CamSuggestReply>(), &run(&mut s, "cam.suggest", json!({"tools": tools})));
    let mut strict = Session { param_check: ParamCheck::Strict, ..Session::default() };
    let e = strict.execute("cam.features", &json!({"axes": "z"})).unwrap_err();
    assert!(e.to_string().contains("axes"), "{e}");
}
