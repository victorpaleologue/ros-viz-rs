use anyhow::{Error, Result};
use ros2_client::{
    Context, MessageTypeName, Name, NodeName, NodeOptions,
    ros2::{
        Duration, QosPolicyBuilder,
        policy::{Durability, History, Reliability},
    },
};
use std::{env, fs, path::PathBuf};

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    // Get output path from args or use default
    let args: Vec<String> = env::args().collect();
    let output_path = if args.len() > 1 {
        let output_path = PathBuf::from(&args[1]);
        println!("Will save URDF to: {}", output_path.display());
        Some(output_path)
    } else {
        println!("Will display output path to stdout");
        None
    };

    // Get domain ID from environment or default to 0
    let domain_id: u32 = env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    println!("Connecting to ROS2 domain {}", domain_id);

    // Create ROS2 context and node
    let ctx = Context::new()?;
    let node_name = NodeName::new("/", "robot_description_collector")?;
    let mut node = ctx.new_node(node_name, NodeOptions::new())?;

    println!("Subscribing to /robot_description...");
    // QoS for latched topics (transient local durability)
    // This allows us to receive the last published message even if it was sent before we subscribed
    let qos = QosPolicyBuilder::new()
        .durability(Durability::TransientLocal)
        .history(History::KeepLast { depth: 1 })
        .reliability(Reliability::Reliable {
            max_blocking_time: Duration::from_secs(1),
        })
        .build();

    // The /robot_description topic.
    let robot_description_topic = node.create_topic(
        &Name::new("/", "robot_description")?,
        MessageTypeName::new("std_msgs", "String"),
        &qos,
    )?;

    // Create subscription
    let subscription = node.create_subscription::<String>(&robot_description_topic, None)?;

    let node_spinner = node.spinner()?;

    println!("Waiting for /robot_description message...");
    println!("Press Ctrl+C to exit\n");

    tokio::select! {
        _ = node_spinner.spin() => { Err(Error::msg("ROS node spin stopped"))}
        // Try to take a message
        res = subscription.async_take() => {
            match res {
                Ok((msg, _info)) => {
                    println!("✓ Found robot_description. Length: {} bytes", msg.len());

                    // Check if it looks like valid XML
                    if msg.trim().starts_with("<?xml") || msg.trim().starts_with("<robot") {
                        println!("✓ Content looks like valid URDF XML");

                        // If option set to save the file
                        if let Some(output_path) = output_path {
                            // Print first characters
                            let preview_len = 200;
                            let preview = if msg.len() > preview_len {
                                format!("{}...", &msg[..preview_len])
                            } else {
                                msg.clone()
                            };
                            println!("Content preview:\n{preview}\n");

                            if let Some(parent) = output_path.parent() {
                                fs::create_dir_all(parent)?;
                            }
                            fs::write(&output_path, &msg)?;
                            println!("✓ Saved URDF to: {}", output_path.display());
                        }
                        // If no option to save the file: display it fully
                        else {
                            println!("Full content:\n{msg}\n");
                        }
                    } else {
                        return Err(Error::msg(format!("⚠ Content doesn't look like URDF XML")));
                    }

                    println!("\n✓ Test completed successfully!");
                    Ok(())
                }
                Err(err) => Err(Error::msg(format!(
                    "Error reading /robot_description: {err:?}"
                ))),
            }
        }
    }
}
