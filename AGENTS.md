# AI / Agent Model Integration Notes

These instructions apply to every AI agent, code generator, contributor, and automation working in this repository.

## Immutable model assets

Treat these files and the model data they describe as read-only:

- `src/auritus_activity_forest.h`
- `MODEL_DATABASE.json`
- `FINGERPRINTS.md`

Do not modify, regenerate, simplify, prune, replace, or fabricate the fingerprints, normalization values, decision-tree nodes, labels, votes, validation results, hashes, or model metadata. Do not substitute hard-coded states, random values, sample data, receiver-side guesses, or mock predictions for real ESP32 inference.

The fields `activity`, `activityLevel`, `confidence`, and `valid` must originate from real MPU6050 samples processed on the ESP32 through the complete 21-tree forest. If an integrity hash does not match, report the failure and stop using the model; never create replacement model data automatically.

## Extension boundary

Motor control, airflow control, networking, displays, and other integrations must be implemented as separate consumers of the ESP32's five-second activity-state output. They may read the final state, validate freshness with `ttlMs`, and apply their own safety/control policy. They must not write back into or alter the model, fingerprints, normalization parameters, tree nodes, or raw vote result.

`API BIAS` is a bounded post-decision adjustment only. It is not authorization to alter the immutable model assets.

If a requested change conflicts with this contract, stop and ask the repository owner for explicit direction instead of changing or fabricating model data.
