# Frank CUDA Progress Deck Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and validate a concise PowerPoint progress deck for the August 17 meeting with Frank.

**Architecture:** Generate the deck from a single PptxGenJS source file with reusable layout helpers, fixed benchmark data arrays, and native PowerPoint charts/tables. Render the generated deck to PDF and slide images, inspect every slide, fix visual defects, and rerun content and visual checks.

**Tech Stack:** Node.js, PptxGenJS, LibreOffice, Poppler, Python/Pillow for montage generation.

---

### Task 1: Build the deck source

**Files:**
- Create: `docs/presentations/2026-08-17-cuda-shared-solver-progress-update.js`
- Create: `docs/presentations/2026-08-17-cuda-shared-solver-progress-update.pptx`

- [ ] **Step 1: Define the theme and reusable slide helpers**

Create helpers for titles, footers, rounded cards, architecture arrows,
tables, and domain speedup charts. Use a 16:9 layout, navy/off-white
backgrounds, CUDA green, amber, and violet accents.

- [ ] **Step 2: Encode the validated benchmark data**

Encode the 16-workload general campaign, the three-way BAL comparison, the
specialized SFM matrix, and the Pose2 initialization controls directly from
the reviewed result reports.

- [ ] **Step 3: Create six main slides and six appendix slides**

Create the exact slide sequence from the approved design. Keep main-slide
body text at 16 pt or larger and appendix tables at 11 pt or larger.

- [ ] **Step 4: Generate the PowerPoint**

Run:

```bash
node docs/presentations/2026-08-17-cuda-shared-solver-progress-update.js
```

Expected: the `.pptx` file is created without an exception.

### Task 2: Validate content and rendering

**Files:**
- Inspect: `docs/presentations/2026-08-17-cuda-shared-solver-progress-update.pptx`
- Create temporarily: `/tmp/cuda-progress-deck-review/*`

- [ ] **Step 1: Convert the deck to PDF and slide images**

Run LibreOffice headless conversion followed by `pdftoppm -jpeg -r 150`.
Expected: one PDF and one image for every slide.

- [ ] **Step 2: Perform content QA**

Extract slide text using the available Office/PDF tools. Confirm the title,
all six main-slide headings, appendix headings, all 16 workloads, and all
three backend names are present. Search for `TODO`, `TBD`, `xxxx`, `lorem`,
and `ipsum`; expect no matches.

- [ ] **Step 3: Perform visual QA**

Create a montage and inspect individual full-resolution slide images for
overlap, clipping, insufficient margins, low contrast, and unreadable tables.
Record at least one concrete improvement from the first pass.

- [ ] **Step 4: Fix and re-render**

Patch the JavaScript source, regenerate the deck, and re-render every affected
slide. Repeat until the new pass reveals no additional defects.

### Task 3: Final verification

**Files:**
- Verify: `docs/presentations/2026-08-17-cuda-shared-solver-progress-update.js`
- Verify: `docs/presentations/2026-08-17-cuda-shared-solver-progress-update.pptx`

- [ ] **Step 1: Verify file integrity and slide count**

Confirm LibreOffice can reopen and convert the final deck and that the deck
contains the expected 12 slides.

- [ ] **Step 2: Verify repository hygiene**

Run:

```bash
git diff --check
```

Expected: no output and exit status zero.

- [ ] **Step 3: Deliver the deck**

Provide a clickable link to the `.pptx`, identify the six-slide main story,
and mention that complete numeric tables are in the appendix.

