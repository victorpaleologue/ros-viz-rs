//! List all ROS 2 topics discovered via DDS.
//!
//! Uses the same DDS endpoint detection events as the `populate_topics` system:
//! `WriterDetected` and `ReaderDetected` events from the node's status receiver.
//!
//! Exits when no new topic has been found for 1 second after the first discovery,
//! or after 10 seconds if nothing is discovered at all.
//!
//! # Usage
//!
//! ```bash
//! cargo run --example list_topics
//! # or with a specific domain:
//! ROS_DOMAIN_ID=42 cargo run --example list_topics
//! ```

use std::collections::BTreeMap;
use std::time::{Duration, Instant};
use std::{env, thread};

use ros2_client::rustdds::DomainParticipantStatusEvent;
use ros2_client::{Context, ContextOptions, NodeEvent, NodeName, NodeOptions};

use ros_viz_rs::ros_plugin::{TopicKind, dds_type_to_ros_type, topic_kind_from_dds_name};

/// Maximum time to wait when no topics are discovered at all.
const NO_TOPIC_TIMEOUT: Duration = Duration::from_secs(20);

/// After the first topic is discovered, wait this long without any new
/// discoveries before considering the list complete.
const SETTLE_TIMEOUT: Duration = Duration::from_secs(10);

/// Poll interval between event drain rounds.
const POLL_INTERVAL: Duration = Duration::from_millis(50);

/// A discovered topic entry (keyed by raw DDS topic name).
struct TopicEntry {
    type_name: String,
    kind: TopicKind,
    has_writers: bool,
    has_readers: bool,
}

fn main() {
    let domain_id: u16 = env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    println!("Discovering topics on DDS domain {domain_id} ...");

    let ctx = Context::with_options(ContextOptions::new().domain_id(domain_id))
        .expect("failed to create ROS context");
    let node_name = NodeName::new("/", "list_topics").expect("invalid node name");
    let mut node = ctx
        .new_node(node_name, NodeOptions::new())
        .expect("failed to create ROS node");

    let spinner = node.spinner().expect("failed to create spinner");
    let event_rx = node.status_receiver();

    // Spin the node in a background thread so DDS discovery runs.
    thread::Builder::new()
        .name("ros_spinner".into())
        .spawn(move || {
            let _ = futures::executor::block_on(spinner.spin());
        })
        .expect("failed to spawn spinner thread");

    let mut topics: BTreeMap<String, TopicEntry> = BTreeMap::new();
    let start = Instant::now();
    let mut last_new_topic = None::<Instant>;

    loop {
        // Drain all pending events.
        while let Ok(event) = event_rx.try_recv() {
            let NodeEvent::DDS(dds_event) = event else {
                continue;
            };
            match dds_event {
                DomainParticipantStatusEvent::WriterDetected { writer } => {
                    let entry = topics.entry(writer.topic_name.clone()).or_insert_with(|| {
                        last_new_topic = Some(Instant::now());
                        TopicEntry {
                            type_name: dds_type_to_ros_type(&writer.type_name),
                            kind: topic_kind_from_dds_name(&writer.topic_name),
                            has_writers: false,
                            has_readers: false,
                        }
                    });
                    entry.has_writers = true;
                }
                DomainParticipantStatusEvent::ReaderDetected { reader } => {
                    let entry = topics.entry(reader.topic_name.clone()).or_insert_with(|| {
                        last_new_topic = Some(Instant::now());
                        TopicEntry {
                            type_name: dds_type_to_ros_type(&reader.type_name),
                            kind: topic_kind_from_dds_name(&reader.topic_name),
                            has_writers: false,
                            has_readers: false,
                        }
                    });
                    entry.has_readers = true;
                }
                _ => {}
            }
        }

        // Check timeouts.
        if let Some(last) = last_new_topic {
            if last.elapsed() >= SETTLE_TIMEOUT {
                break;
            }
        } else if start.elapsed() >= NO_TOPIC_TIMEOUT {
            println!("No topics discovered after {NO_TOPIC_TIMEOUT:?}.");
            return;
        }

        thread::sleep(POLL_INTERVAL);
    }

    // Print results grouped by kind.
    println!("\nDiscovered {} topic(s):\n", topics.len());

    for (dds_name, entry) in &topics {
        let dir = match (entry.has_writers, entry.has_readers) {
            (true, true) => "pub/sub",
            (true, false) => "pub",
            (false, true) => "sub",
            (false, false) => "?",
        };
        let kind_label = match &entry.kind {
            TopicKind::Normal(name) => format!("topic  {name}"),
            TopicKind::ServiceRequest(name) => format!("svc-rq {name}"),
            TopicKind::ServiceReply(name) => format!("svc-rr {name}"),
            TopicKind::Action(name) => format!("action {name}"),
            TopicKind::Unknown => format!("???    {dds_name}"),
        };
        println!("  [{dir:>7}] {kind_label}  ({})", entry.type_name);
    }
}
