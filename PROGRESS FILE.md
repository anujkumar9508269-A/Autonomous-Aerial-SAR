# Progress Tracker — Autonomous Aerial SAR (PS-3)

**Team IVAR 1 | IITISoC 2026**

---

## Weekly Milestones

| Week | Targets Achieved |
|---|---|
| 1 | Environment setup (ROS 2 Humble, Gazebo 11, ArduPilot SITL, MAVROS). First autonomous SITL flight — takeoff, 10s hover, safe landing verified. |
| 2 | Literature review and approach finalised for all three modules. System architecture designed. ROS 2 node graph defined. |
| 3 | Core implementation complete — lawnmower, non-convex, and spiral coverage strategies; waypoint follower v3.3 with PI controller; 18-check regression suite passing. YOLOv8n model trained on VisDrone and HERIDAL datasets and fine-tunned the hyperparameters of the detection node. Implemented geotagger.py , position_estimator.py , results_logger.py.|

| 4 | Integration — full pipeline (coverage planner + YOLOv8 detection + geo-tagger) launched together. Geo-tagged CSV output produced from autonomous SITL flight. |

---

## Mid-Evaluation Results

| Metric | Target | Achieved |
|---|---|---|
| Area coverage | ≥ 80% | 100% |
| Planning time | — | < 500 ms |
| Detection recall | ≤ 70% | Achieved |
| False positives | ≤ 3 per run | ~2 per run |
| Geo-tag error | ≤ 3 m | 1–3 m |
| Detection FPS | ≥ 10 FPS | Achieved |
| Pipeline autonomy | Partial | Partial |

### Pipeline Autonomy
- `nav_msgs/Path` published on `/coverage_path` at 1 Hz
- Waypoint follower autonomously arms, takes off, follows path, and lands via MAVROS
- YOLO detection node running at ≥ 10 FPS
- Geo-tagger produces raw tags via ray-cast projection
- Position estimator refines estimates via running mean per track ID
- Results logger writes CSV + GeoJSON on mission shutdown

---

## Week 5 Plan — Integration + Tuning

- [ ] Full pipeline integration — we have to build a complete single launch file and resolve the Gazebo startup issue along with the communication issues between ArduPilot SITL and MAVROS.
- [ ] Maximum safe altitude determination — measure YOLO detection recall vs. altitude in simulation to find the optimal flight altitude that balances coverage efficiency and detection reliability
- [ ] Training on higher model — we will train a YOLOv8m model for better results.
- [ ] Implementation of Kalman filter — instead of using the mean of all geotag positions of the same person across multiple detections, we will apply a Kalman filter to get a single position estimate.

---

## Week 6 Plan — Final Deliverables

- [ ] Final technical report (PDF)
- [ ] Demo video (3–5 min) showing full autonomous pipeline
- [ ] Final presentation slides
- [ ] Clean up README, add architecture diagram to `/docs`
- [ ] Tag final release on GitHub
- [ ] Implementation of Kalman filter — instead of using the mean of all geotag positions of the same person across multiple detections, we will apply a Kalman filter to get a single position estimate.

---

## End-Evaluation Targets

| Criterion | Target |
|---|---|
| Coverage | ≥ 95% |
| Detection recall | ≥ 85% |
| False positives | ≤ 1 per run |
| Geo-tag error | ≤ 1 m |
| Detection FPS | ≥ 10 FPS |
| Pipeline autonomy | Fully autonomous |

---

## Known Issues

| Issue | Status |
|---|---|
| False negatives (leaving a person undetected) and assigning different IDs to the same person when detected across different frames | Week 5 |
| Maximum safe altitude for detection not yet determined | Week 5 |
| The current averaging-based position estimator weights all observations equally, regardless of observation geometry (e.g. drone altitude, pixel distance from centre). Because of this, erroneous measurements contribute equally with correct ones, which is why our geo-tag error currently falls in the 1–3 m range. We will apply a Kalman filter with observations weighted according to observation geometry to improve the accuracy of geo-tagged positions. | Week 5–6 |
