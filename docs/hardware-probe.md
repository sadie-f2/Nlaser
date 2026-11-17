# Hardware probe: edge cases on a real Ruida

`tools/laser_edge_cases.sh --serial DEV | --host IP` walks through the protocol
questions we cannot answer from MeerK40t's source. The laser cannot fire: only
memory reads, laser-off dry runs (no cut command, every power 0), goto/jog and
stop are sent, and the temporary `lasers.conf` sets `max_power 0`. The head does
move. Everything is logged with timestamps and exit codes in `laser-edge-DATE/`.

Test-only CLI support it uses:

- `ncad-laser raw HEX WHERE`: one read-only command. Only `DA 00 hh ll` and the
  ENQ `CE` are accepted; anything else is refused before a port is opened.
- `ncad-laser dryrun ... --corrupt-sum`: flips one bit of the file sum. Dry run
  only; the flag is refused on every other command, and `send` has no such path.
- `ncad-laser frame|dryrun ... --wait`: after the upload, poll until the head
  has been still for 1 s and report how long that took.
- `--chunk N` (existing): packet payload size, UDP.

| Step | Question it answers |
|------|---------------------|
| a baseline | Do counters, status and the full settings dump answer on this firmware (RDLC-V8.01.50)? Does the firmware string come back over serial, where replies have no framing? What are the memory values (saved to `settings.txt`)? |
| b idle listen | Does the controller push anything by itself when nothing is happening (UDP and serial)? |
| c reads mid-job | Do status reads answer while the head moves, how slow, and do state, position and file progress change? Over serial a second process cannot hold the port, so status is read right after the upload instead. |
| d second job | What does the controller do with a job that arrives while one runs: NAK, ERR, ignore, queue, or interleave? |
| e stop mid-job | Does D8 01 halt the head at once, and what do state and position read afterwards? |
| f bad file sum | Is a wrong Set File Sum rejected (an error reply, a panel message) or run anyway? Decides whether the sum is a real integrity check to rely on. |
| g chunk sweep | UDP only. Which packet sizes (200/500/1000/1400) are accepted, and how upload time scales. Picks the default `max_packet`. |
| h position check | Do goto and jog land where status says, in which direction does bed +x go on a top-right-origin machine, and is the readback consistent after each move? |
| i unplug | What does the tool print when the link disappears mid-watch, does it exit cleanly rather than hang, and does a fresh run work after reconnecting? |
| j lid open | Does the interlock make the controller refuse a job, report an error, or move anyway? Informs what a front end must say about the lid. |

After the run, give Claude the whole log directory plus what the panel showed,
beeped or did, which the log cannot capture.
