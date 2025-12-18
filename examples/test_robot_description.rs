use anyhow::Result;
use ros2_client::ros2::{
    Duration as RosDuration, QosPolicyBuilder,
    policy::{Durability, History, Reliability},
};
use ros2_client::{Context, MessageTypeName, Name, NodeName, NodeOptions};
use std::env;
use std::time::Duration;

fn main() -> Result<()> {
    // Get domain ID from environment or default to 0
    let domain_id: u32 = env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);

    println!("Connecting to ROS2 domain {}", domain_id);

    // Create ROS2 context and node
    let ctx = Context::new()?;
    let node_name = NodeName::new("/", "robot_description_tester")?;
    let mut node = ctx.new_node(node_name, NodeOptions::new())?;

    println!("Creating subscription to /robot_description with TRANSIENT_LOCAL QoS...");
    println!("(This allows us to receive messages published before we subscribed)\n");

    // Create QoS for latched topics (transient local durability)
    // This allows us to receive the last published message even if it was sent before we subscribed
    let qos = QosPolicyBuilder::new()
        .durability(Durability::TransientLocal)
        .history(History::KeepLast { depth: 1 })
        .reliability(Reliability::Reliable {
            max_blocking_time: RosDuration::from_secs(1),
        })
        .build();

    // Create topic for robot_description
    let robot_description_topic = node.create_topic(
        &Name::new("/", "robot_description")?,
        MessageTypeName::new("std_msgs", "String"),
        &qos,
    )?;

    // Create subscription
    let subscription = node.create_subscription::<String>(&robot_description_topic, None)?;

    println!("Waiting for /robot_description message...");
    println!("Press Ctrl+C to exit\n");

    let timeout = Duration::from_secs(30);
    let start = std::time::Instant::now();
    let mut received_count = 0;

    loop {
        // Try to take a message
        match subscription.take() {
            Ok(Some((msg, _info))) => {
                received_count += 1;
                println!("✓ Received robot_description message #{}", received_count);
                println!("Length: {} bytes", msg.len());

                // Print first 500 characters
                let preview = if msg.len() > 500 {
                    format!("{}...", &msg[..500])
                } else {
                    msg.clone()
                };
                println!("Content preview:\n{}\n", preview);

                // Check if it looks like valid XML
                if msg.trim().starts_with("<?xml") || msg.trim().starts_with("<robot") {
                    println!("✓ Content looks like valid URDF XML");
                } else {
                    println!("⚠ Content doesn't look like XML");
                }

                println!("\n✓ Test completed successfully!");
                break;
            }
            Ok(None) => {
                // No message available yet
                if start.elapsed() > timeout && received_count == 0 {
                    println!(
                        "\n⚠ Timeout: No messages received after {} seconds",
                        timeout.as_secs()
                    );
                    println!(
                        "Make sure a ROS2 node is publishing to /robot_description on domain {}",
                        domain_id
                    );
                    break;
                }
            }
            Err(err) => {
                eprintln!("Error reading /robot_description: {:?}", err);
            }
        }

        // Sleep briefly to avoid busy waiting
        std::thread::sleep(Duration::from_millis(100));
    }

    Ok(())
}
