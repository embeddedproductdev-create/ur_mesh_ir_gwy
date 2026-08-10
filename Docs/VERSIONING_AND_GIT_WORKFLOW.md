# Versioning & Git Workflow — Unimation BLE Mesh Firmware

**Repo:** `ur_mesh_ir_gwy`
**Applies to:** ESP32-S3 Gateway/Node firmware (`IS_GWY` compile-time flag), shared codebase

This document describes how we version firmware releases and how the git
repository should be used day-to-day. Anyone working on this project —
now or in future — should follow this so that any merged binary can always
be traced back to an exact commit, and history stays clean enough to be
useful.

---

## 1. Versioning scheme

We use **git tags** as the single source of truth for version numbers —
not a hardcoded version string in a header file. This works because
ESP-IDF automatically embeds the output of `git describe --tags --dirty`
into the compiled firmware as its version string (visible at boot in the
serial log, e.g. `App "IR_BLE_MESH" version: V1.0.4-1-g468785a6`).

**Tag format:** `v<major>.<minor>.<patch>`

| Segment | Bump when... | Example |
|---|---|---|
| **major** | A breaking change — e.g. incompatible MQTT packet format, mesh re-provisioning required for all existing devices | v1 → v2 |
| **minor** | A milestone feature ships — mesh node-count expansion, energy meter integration, GPIO subsystem, new Modbus function codes | v1.0 → v1.1 |
| **patch** | A bug fix within an already-shipped milestone, no new functionality | v1.0.4 → v1.0.5 |

**Existing tags in this repo** (for reference): `v1.0.0`, `v1.0.1`, `v1.0.3`, `V1.0.4`.
*(Note: keep casing consistent going forward — use lowercase `v` for all new tags.)*

### Why git tags instead of a version macro?
- One less thing to remember to bump before a release
- Impossible for the embedded version string to drift out of sync with what's actually in git
- `merge_release.py` (see §4) uses this same string to name output folders, so the folder name, the firmware's own reported version, and the git history all agree automatically

---

## 2. Commit message conventions

Keep messages **factual and specific** — they become permanent history and
are the fastest way (for you in six months, or a teammate) to find when/why
something changed.

**Good:**
```
Fix ElementAddr validation to reject empty string, not just wrong type
Add FC04 Modbus read for energy meter, push-model reporting via esp_timer
Bump BLE_MESH_MAX_PROV_NODES to 24, adjust CRPL and MSG_CACHE_SIZE
```

**Avoid:**
- Vague messages: `fix stuff`, `wip`, `testing`
- Internal-only context mixed into permanent history if the repo may ever
  be shared with the client (keep debugging narration in your own notes,
  not the commit log)

One commit = one logical change where practical. It's fine to commit more
often during active debugging; squash before tagging a release if the
history around it is noisy (`git rebase -i`), but this is optional — don't
let commit hygiene block progress.

---

## 3. Branching

Current practice: **single `main` branch**, direct commits, tagged at
milestones. This is appropriate given the team size (you + one hardware
partner, no parallel firmware contributors).

If a second person starts contributing firmware code, or if long-running
experimental work (e.g. GPIO subsystem) risks destabilizing `main` for
multiple days, switch to short-lived feature branches:

```bash
git checkout -b feature/gpio-subsystem
# ... work, commit ...
git checkout main
git merge feature/gpio-subsystem
git branch -d feature/gpio-subsystem
```

Don't over-engineer this until it's actually needed.

---

## 4. Release process — from code change to client-ready binaries

### Step A — Develop and test (no versioning yet)
Build normally, flash, and iterate using `merge_test.py`:
```bash
idf.py build
python merge_test.py GWY00001      # merge + flash one device to test quickly
```
`merge_test.py` always writes to `merged_bin_test/` and overwrites on every
run — it is **not** for client delivery, and has no git awareness.

### Step B — Once a build is confirmed working on hardware
```bash
git add -A
git status                          # sanity check: build/, merged_bin*/, .zip, etc. should NOT appear
git commit -m "Add FC04 Modbus read for energy meter"
```

### Step C — Tag the release
```bash
git tag -a v1.1.0 -m "Energy meter integration, 24-node mesh capacity"
git push origin main
git push origin v1.1.0
```

### Step D — Generate the versioned, client-ready merged binaries
```bash
python merge_release.py
```
This will:
- Refuse to run if the working tree is dirty (uncommitted changes) — this
  is intentional, so a client build always maps to a real commit
- Create `merged_bin/v1.1.0/` containing one `merged-<SERIAL>.bin` per
  device serial number, plus a `manifest.json` recording the git commit
  hash, branch, build timestamp, flash settings, and which devices were
  merged

### Step E — Hand off to client / manufacturing
Zip and share the whole `merged_bin/v1.1.0/` folder. Everything needed to
know exactly what's inside it is in `manifest.json`.

---

## 5. Repository layout (what's tracked vs. ignored)

| Path | Tracked? | Why |
|---|---|---|
| `main/`, `components/` (source) | ✅ Yes | Source code |
| `sdkconfig` | ✅ Yes | Fixed config (24-node mesh values, partition table selection, etc.) — not `sdkconfig.defaults`, so the exact config matters |
| `ir_partitions.csv` | ✅ Yes | Partition table definition |
| `SerNoGen.py`, `nvs_partition_gen.py` | ✅ Yes | Serial number generation tooling |
| `merge_test.py`, `merge_release.py` | ✅ Yes | Build/release tooling |
| `.gitignore` | ✅ Yes | — |
| `build/` | ❌ No | Regenerated by `idf.py build` |
| `merged_bin/`, `merged_bin_test/` | ❌ No | Regenerated from source + serial bins |
| `main/src/NVS_Partition_Generator/bin/` | ❌ No | Per-device generated serial NVS binaries — not source |
| `.metadata/`, `.settings/` | ❌ No | Espressif IDE workspace state, machine-specific |
| `*.zip` | ❌ No | Release archives — do not belong in git history (large, regenerable) |

If you're ever unsure whether something should be tracked, ask: *"Can this
be regenerated from something else already in the repo?"* If yes, ignore it.

---

## 6. Handling large files

GitHub rejects any single file over 100 MB. **Never commit `.zip` archives,
merged firmware binaries, or other large build outputs** — they're already
excluded via `.gitignore`. If a large file is committed by mistake:

- **If it's only in the most recent commit and not yet pushed:**
  ```bash
  git rm --cached <file>
  git commit --amend --no-edit
  ```
- **If it's already spread across history / already pushed:** this requires
  rewriting history with `git-filter-repo` — a more involved, higher-risk
  operation. Back up the repo folder first, and don't attempt this without
  confirming nobody else has a clone of the affected commits.

---

## 7. Quick reference — common commands

```bash
# See history
git log --oneline

# See all tags
git tag

# Check what a tag points to
git show v1.1.0 --stat

# Confirm current version string (same as what firmware will embed)
git describe --tags --always --dirty

# Merge one device for a quick test
python merge_test.py GWY00001

# Full versioned release
python merge_release.py

# Force a release build even with uncommitted changes (marks folder as -dirty)
python merge_release.py --force
```

---

*Keep this document updated as the workflow evolves — it's more useful
living in the repo (`docs/VERSIONING_AND_GIT_WORKFLOW.md`) than as a
standalone file, so it stays in sync with the tooling it describes.*
