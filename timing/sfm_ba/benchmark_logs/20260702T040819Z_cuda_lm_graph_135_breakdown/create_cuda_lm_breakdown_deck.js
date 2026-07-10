const fs = require("fs");
const path = require("path");
const pptxgen = require("pptxgenjs");

const pptx = new pptxgen();
pptx.defineLayout({ name: "WIDE", width: 13.333, height: 7.5 });
pptx.layout = "WIDE";
pptx.author = "cu-gtsam benchmark";
pptx.company = "GTSAM CUDA SFM";
pptx.subject = "CUDA SFM LM progress and hybrid baseline feasibility";
pptx.title = "CUDA SFM LM progress";
pptx.lang = "en-US";
pptx.theme = {
  headFontFace: "Aptos Display",
  bodyFontFace: "Aptos",
  lang: "en-US",
};

const C = {
  bg: "F7F8FA",
  ink: "18212F",
  muted: "5A6472",
  faint: "E7EBF0",
  navy: "1F2A44",
  teal: "0E7C86",
  green: "46A06F",
  amber: "F2A23A",
  coral: "D95D54",
  blue: "3E74B8",
  white: "FFFFFF",
  paleTeal: "EAF6F7",
  paleGreen: "EAF7EF",
  paleAmber: "FEF4E3",
  paleCoral: "FBEDEB",
};

const W = 13.333;
const H = 7.5;
const M = 0.58;
const OUTPUT_DIR = path.resolve(__dirname, "../20260710T_cuda_sfm_progress");
const OUTPUT_FILE = path.join(OUTPUT_DIR, "cuda_sfm_progress_update.pptx");

function addBg(slide, dark = false) {
  slide.background = { color: dark ? C.navy : C.bg };
  if (!dark) {
    slide.addShape(pptx.ShapeType.rect, {
      x: 0, y: 0, w: W, h: 0.16,
      fill: { color: C.teal }, line: { color: C.teal },
    });
  }
}

function addTitle(slide, title, subtitle, dark = false) {
  const ink = dark ? C.white : C.ink;
  const muted = dark ? "D9E5EA" : C.muted;
  slide.addText(title, {
    x: M, y: 0.36, w: 10.7, h: 0.5, margin: 0,
    fontFace: "Aptos Display", fontSize: 29, bold: true, color: ink, fit: "shrink",
  });
  if (subtitle) {
    slide.addText(subtitle, {
      x: M, y: 0.91, w: 11.5, h: 0.28, margin: 0,
      fontSize: 14.5, color: muted, fit: "shrink",
    });
  }
}

function addFooter(slide, idx, text = "CUDA SFM LM progress | Dubrovnik 135") {
  slide.addText(`${text} | ${idx}`, {
    x: M, y: 7.14, w: 7.8, h: 0.15, margin: 0,
    fontSize: 8.5, color: "7B8490", fit: "shrink",
  });
}

function addBadge(slide, text, x, y, color) {
  const width = Math.max(0.96, 0.09 * text.length + 0.45);
  slide.addShape(pptx.ShapeType.rect, {
    x, y, w: width, h: 0.3,
    fill: { color }, line: { color },
  });
  slide.addText(text, {
    x: x + 0.08, y: y + 0.075, w: width - 0.16, h: 0.11, margin: 0,
    fontSize: 9.5, bold: true, color: C.white, align: "center", fit: "shrink",
  });
  return width;
}

function addPanel(slide, x, y, w, h, accent = C.teal, fill = C.white) {
  slide.addShape(pptx.ShapeType.rect, {
    x, y, w, h, fill: { color: fill }, line: { color: C.faint, width: 0.75 },
  });
  slide.addShape(pptx.ShapeType.rect, {
    x, y, w: 0.07, h, fill: { color: accent }, line: { color: accent },
  });
}

function addCallout(slide, label, value, note, x, y, w, color, dark = false) {
  const fill = dark ? "263956" : C.white;
  const ink = dark ? C.white : C.ink;
  const muted = dark ? "D9E5EA" : C.muted;
  addPanel(slide, x, y, w, 1.07, color, fill);
  slide.addText(value, {
    x: x + 0.2, y: y + 0.13, w: w - 0.34, h: 0.34, margin: 0,
    fontSize: 25, bold: true, color, fit: "shrink",
  });
  slide.addText(label, {
    x: x + 0.2, y: y + 0.57, w: w - 0.34, h: 0.16, margin: 0,
    fontSize: 13.5, bold: true, color: ink, fit: "shrink",
  });
  slide.addText(note, {
    x: x + 0.2, y: y + 0.81, w: w - 0.34, h: 0.13, margin: 0,
    fontSize: 10.5, color: muted, fit: "shrink",
  });
}

function addStackedBar(slide, items, x, y, w, h, total, labelThreshold = 0.9) {
  let cursor = x;
  items.forEach((item) => {
    const segment = Math.max(0.02, w * item.value / total);
    slide.addShape(pptx.ShapeType.rect, {
      x: cursor, y, w: segment, h,
      fill: { color: item.color }, line: { color: C.white, width: 0.7 },
    });
    if (segment >= labelThreshold) {
      slide.addText(item.short || `${item.pct}%`, {
        x: cursor + 0.06, y: y + h / 2 - 0.1, w: segment - 0.12, h: 0.16, margin: 0,
        fontSize: 11, bold: true, color: C.white, align: "center", fit: "shrink",
      });
    }
    cursor += segment;
  });
}

function addLegend(slide, items, x, y, columns = 2, width = 3.0) {
  items.forEach((item, index) => {
    const col = index % columns;
    const row = Math.floor(index / columns);
    const px = x + col * width;
    const py = y + row * 0.34;
    slide.addShape(pptx.ShapeType.rect, {
      x: px, y: py + 0.055, w: 0.14, h: 0.14,
      fill: { color: item.color }, line: { color: item.color },
    });
    slide.addText(item.label, {
      x: px + 0.22, y: py, w: width - 0.28, h: 0.21, margin: 0,
      fontSize: 11.5, color: C.ink, fit: "shrink",
    });
  });
}

function addTable(slide, rows, x, y, colWidths, options = {}) {
  const rowH = options.rowH || 0.38;
  const fontSize = options.fontSize || 12;
  const headerFill = options.headerFill || C.navy;
  rows.forEach((row, r) => {
    let cursor = x;
    const fill = r === 0 ? headerFill : (r % 2 ? C.white : "F0F4F7");
    const color = r === 0 ? C.white : C.ink;
    row.forEach((cell, c) => {
      const align = options.alignments && options.alignments[c] ? options.alignments[c] : (c === 0 ? "left" : "right");
      slide.addShape(pptx.ShapeType.rect, {
        x: cursor, y: y + r * rowH, w: colWidths[c], h: rowH,
        fill: { color: fill }, line: { color: C.bg, width: 0.45 },
      });
      slide.addText(String(cell), {
        x: cursor + 0.07, y: y + r * rowH + 0.095, w: colWidths[c] - 0.14, h: rowH - 0.15, margin: 0,
        fontSize, bold: r === 0 || (options.boldFirst !== false && c === 0), color, align, fit: "shrink",
      });
      cursor += colWidths[c];
    });
  });
}

function addBarRows(slide, rows, x, y, labelW, barW, maxValue, unit = "ms") {
  rows.forEach((row, index) => {
    const py = y + index * 0.48;
    slide.addText(row.label, {
      x, y: py + 0.03, w: labelW, h: 0.2, margin: 0,
      fontSize: 13, bold: true, color: C.ink, fit: "shrink",
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: x + labelW + 0.12, y: py + 0.045, w: barW, h: 0.18,
      fill: { color: "DEE5EB" }, line: { color: "DEE5EB" },
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: x + labelW + 0.12, y: py + 0.045, w: Math.max(0.03, barW * row.value / maxValue), h: 0.18,
      fill: { color: row.color }, line: { color: row.color },
    });
    slide.addText(`${row.value.toFixed(3)} ${unit}`, {
      x: x + labelW + barW + 0.3, y: py + 0.015, w: 1.25, h: 0.19, margin: 0,
      fontSize: 12, bold: true, color: row.color, align: "right", fit: "shrink",
    });
  });
}

function addPipeline(slide, stages, x, y, w, h, accent = C.teal) {
  const gap = 0.18;
  const stageW = (w - gap * (stages.length - 1)) / stages.length;
  stages.forEach((stage, index) => {
    const px = x + index * (stageW + gap);
    const color = stage.color || accent;
    slide.addShape(pptx.ShapeType.rect, {
      x: px, y, w: stageW, h,
      fill: { color: stage.fill || C.white }, line: { color: index === 0 ? C.navy : C.faint, width: 0.75 },
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: px, y, w: stageW, h: 0.1,
      fill: { color }, line: { color },
    });
    slide.addText(stage.title, {
      x: px + 0.12, y: y + 0.21, w: stageW - 0.24, h: 0.32, margin: 0,
      fontSize: 15, bold: true, color: C.ink, align: "center", fit: "shrink",
    });
    slide.addText(stage.note, {
      x: px + 0.12, y: y + 0.72, w: stageW - 0.24, h: h - 0.85, margin: 0,
      fontSize: 12.5, color: C.muted, align: "center", valign: "mid", fit: "shrink",
    });
    if (index < stages.length - 1) {
      slide.addShape(pptx.ShapeType.chevron, {
        x: px + stageW + 0.035, y: y + h / 2 - 0.15, w: 0.13, h: 0.3,
        fill: { color: "9AA8B6" }, line: { color: "9AA8B6" },
      });
    }
  });
}

const apiTotal = 426.223;
const apiTop = [
  { label: "Optimizer construction 36.203 ms (8.49%)", value: 36.203, pct: "8.49", color: C.blue, short: "8.5%" },
  { label: "optimize() 376.192 ms (88.26%)", value: 376.192, pct: "88.26", color: C.teal, short: "88.3%" },
  { label: "Result/error queries 13.828 ms (3.24%)", value: 13.828, pct: "3.24", color: C.amber, short: "3.2%" },
];
const optimizeRows = [
  { label: "Graph conversion", value: 114.246, color: C.teal },
  { label: "CUDA backend", value: 137.542, color: C.green },
  { label: "Backend return / assignment", value: 26.685, color: C.blue },
  { label: "Values merge / state update", value: 67.464, color: C.amber },
  { label: "Converted-data destruction", value: 30.247, color: C.coral },
  { label: "Residual", value: 0.008, color: C.muted },
];
const backendRows = [
  { label: "Setup", value: 67.544, color: C.teal },
  { label: "Solve loop", value: 29.968, color: C.green },
  { label: "Download Values", value: 40.030, color: C.amber },
];

// Slide 1
{
  const slide = pptx.addSlide();
  addBg(slide, true);
  addBadge(slide, "Measured", 10.95, 0.47, C.teal);
  addBadge(slide, "Proposed", 12.05, 0.47, C.amber);
  addTitle(slide, "CUDA SFM LM progress", "Timing breakdown and a general-solver baseline", true);
  slide.addText("Dataset: Dubrovnik 135 | dense Schur | 3 LM iterations", {
    x: M, y: 1.38, w: 7.2, h: 0.22, margin: 0,
    fontSize: 15, bold: true, color: "D9E5EA", fit: "shrink",
  });
  addCallout(slide, "Current graph API", "0.426 s", "mean of 3 timed runs", 0.72, 2.12, 2.7, C.teal, true);
  addCallout(slide, "CUDA backend", "0.138 s", "specialized dense-Schur path", 3.72, 2.12, 2.7, C.green, true);
  addCallout(slide, "Baseline question", "Hybrid", "CPU nonlinear work + GPU solve", 6.72, 2.12, 2.95, C.amber, true);
  slide.addText("This update has two parts: where the current API path spends time, then whether a general CPU/GPU separation is plausible.", {
    x: 0.75, y: 3.62, w: 8.9, h: 0.5, margin: 0,
    fontSize: 18, bold: true, color: C.white, fit: "shrink",
  });
  slide.addShape(pptx.ShapeType.rect, {
    x: 10.15, y: 0, w: 3.183, h: H,
    fill: { color: C.teal }, line: { color: C.teal },
  });
  [
    ["1", "Profile current path", "Measured"],
    ["2", "Separate general baseline", "Proposed"],
    ["3", "Measure full LM convergence", "Next"],
  ].forEach((item, index) => {
    const y = 1.45 + index * 1.48;
    slide.addShape(pptx.ShapeType.ellipse, {
      x: 10.58, y, w: 0.5, h: 0.5,
      fill: { color: index === 1 ? C.amber : C.navy }, line: { color: C.white, transparency: 100 },
    });
    slide.addText(item[0], {
      x: 10.58, y: y + 0.15, w: 0.5, h: 0.12, margin: 0,
      fontSize: 12, bold: true, color: C.white, align: "center",
    });
    slide.addText(item[1], {
      x: 11.28, y: y + 0.03, w: 1.62, h: 0.28, margin: 0,
      fontSize: 15, bold: true, color: C.white, fit: "shrink",
    });
    slide.addText(item[2], {
      x: 11.28, y: y + 0.36, w: 1.45, h: 0.15, margin: 0,
      fontSize: 11, color: "D9E5EA", fit: "shrink",
    });
  });
  slide.addNotes("About 30 seconds. This is a factual progress update. First I will show the measured timing for the current specialized CUDA SFM path on Dubrovnik 135 with dense Schur and three LM iterations. Then I will use separate feasibility measurements to ask whether a more general CPU and GPU separation is worth building. The goal is to identify the next experiment, not to claim a finished faster solver.");
}

// Slide 2
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "Current implementation", "The public graph API adapts generic GTSAM objects into a specialized CUDA SFM backend.");
  addBadge(slide, "Measured", 11.65, 0.48, C.teal);
  addPipeline(slide, [
    { title: "NonlinearFactorGraph + Values", note: "Generic graph, current variable state, and factor ownership.", color: C.navy, fill: "F1F4F8" },
    { title: "Specialized graph conversion", note: "Extract SFM factors, keys, measurements, cameras, and points into host arrays.", color: C.teal },
    { title: "CUDA SFM arrays / backend", note: "Pack, upload, run LM with dense Schur, and retain packed result arrays.", color: C.green, fill: C.paleGreen },
    { title: "Downloaded Values", note: "Rebuild optimized camera and point Values from host-side result arrays.", color: C.amber, fill: C.paleAmber },
    { title: "CPU merge", note: "Merge SFM results into the original Values and update optimizer state.", color: C.coral, fill: C.paleCoral },
  ], 0.6, 1.55, 12.1, 2.15);
  slide.addText("Boundary responsibilities", {
    x: 0.72, y: 4.25, w: 3.2, h: 0.23, margin: 0,
    fontSize: 17, bold: true, color: C.ink,
  });
  addTable(slide, [
    ["Boundary", "What it does", "Why it matters"],
    ["Generic to SFM", "CPU representation conversion", "The CUDA backend does not consume arbitrary GTSAM factors."],
    ["SFM to device", "Array packing and backend setup", "Includes host builds as well as GPU work."],
    ["Result to Values", "CPU reconstruction and merge", "Object/state handling is included in end-to-end time."],
  ], 0.72, 4.62, [2.15, 3.85, 5.9], { rowH: 0.47, fontSize: 12.2, headerFill: C.navy, alignments: ["left", "left", "left"] });
  slide.addText("Framing: this profile measures an integration path, not only GPU kernel time.", {
    x: 0.72, y: 6.62, w: 8.9, h: 0.26, margin: 0,
    fontSize: 16, bold: true, color: C.teal, fit: "shrink",
  });
  addFooter(slide, 2);
  slide.addNotes("About 60 seconds. Start from the left: the user-facing input is a normal GTSAM graph and Values, with generic factor ownership and the current variable state. The specialized path scans that graph, selects the SFM subset, and creates arrays for cameras, points, measurements, and key maps. CUDA operates on those arrays, not on arbitrary GTSAM factors. After the backend returns, optimized arrays must be rebuilt as Values and merged into the original state. I am calling out each boundary because the timing includes this integration work, not only the GPU kernel.");
}

// Slide 3
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "Whole graph API: 0.426 s", "Measured scope: warm-up excluded, mean of 3 runs.");
  addBadge(slide, "Measured", 11.65, 0.48, C.teal);
  addCallout(slide, "Graph API total", "426.223 ms", "optimizer + optimize + result/error", 0.72, 1.32, 3.0, C.navy);
  slide.addText("The API denominator includes construction and final result/error queries, not just optimize().", {
    x: 4.1, y: 1.54, w: 7.7, h: 0.32, margin: 0,
    fontSize: 17, bold: true, color: C.ink, fit: "shrink",
  });
  addStackedBar(slide, apiTop, 0.72, 2.45, 11.9, 0.62, apiTotal);
  addLegend(slide, apiTop, 0.72, 3.28, 1, 6.5);
  addTable(slide, [
    ["Component", "Time", "% of graph API"],
    ["Optimizer construction", "36.203 ms", "8.49%"],
    ["optimize()", "376.192 ms", "88.26%"],
    ["Result/error queries", "13.828 ms", "3.24%"],
  ], 0.72, 4.4, [4.6, 2.0, 2.0], { rowH: 0.5, fontSize: 14, headerFill: C.navy });
  addPanel(slide, 9.75, 4.4, 2.75, 1.5, C.teal, C.paleTeal);
  slide.addText("Scope reminder", { x: 10.0, y: 4.66, w: 2.25, h: 0.2, margin: 0, fontSize: 15, bold: true, color: C.teal });
  slide.addText("The following slides unpack the 376.192 ms optimize() call.", {
    x: 10.0, y: 5.03, w: 2.2, h: 0.5, margin: 0,
    fontSize: 15, bold: true, color: C.ink, align: "center", valign: "mid", fit: "shrink",
  });
  addFooter(slide, 3);
  slide.addNotes("About 55 seconds. This is the complete public API timing. Warm-up on a small problem was excluded, and the result is the mean of three timed runs. Construction contributes 36.203 milliseconds, optimize contributes 376.192 milliseconds, and final result and error queries contribute 13.828 milliseconds. Nearly eighty-eight percent is optimize, so the next slides unpack that call. I keep the other pieces visible because they define the graph API scope and prevent a partial number from being presented as end-to-end.");
}

// Slide 4
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "Inside optimize(): 0.376 s", "Measured components. Stacked bar: % of optimize(); table: % of Graph API.");
  addBadge(slide, "Measured", 11.65, 0.48, C.teal);
  const optimizeStack = optimizeRows.map((row) => ({ ...row, pct: (row.value / apiTotal * 100).toFixed(1), short: row.value > 8 ? `${(row.value / 376.192 * 100).toFixed(0)}%` : "" }));
  addStackedBar(slide, optimizeStack, 0.72, 1.62, 11.9, 0.58, 376.192, 1.25);
  addLegend(slide, optimizeRows.map((row) => ({ label: `${row.label} ${row.value.toFixed(3)} ms`, color: row.color })), 0.72, 2.45, 2, 5.65);
  addTable(slide, [
    ["Measured component", "Time", "% of whole API"],
    ["Graph conversion", "114.246 ms", "26.80%"],
    ["CUDA backend", "137.542 ms", "32.27%"],
    ["Backend return / assignment", "26.685 ms", "6.26%"],
    ["Values merge / state update", "67.464 ms", "15.83%"],
    ["Converted-data destruction", "30.247 ms", "7.10%"],
    ["Residual", "0.008 ms", "0.00%"],
  ], 0.72, 3.42, [4.55, 2.0, 2.0], { rowH: 0.39, fontSize: 12.4, headerFill: C.navy });
  addPanel(slide, 9.65, 3.42, 2.86, 2.55, C.coral, C.paleCoral);
  slide.addText("CPU result management", { x: 9.92, y: 3.72, w: 2.35, h: 0.22, margin: 0, fontSize: 15, bold: true, color: C.coral, align: "center" });
  slide.addText("97.711 ms", { x: 9.92, y: 4.15, w: 2.35, h: 0.34, margin: 0, fontSize: 25, bold: true, color: C.coral, align: "center" });
  slide.addText("Values merge plus temporary converted-data destruction", {
    x: 9.92, y: 4.68, w: 2.35, h: 0.55, margin: 0, fontSize: 13, bold: true, color: C.ink, align: "center", valign: "mid", fit: "shrink",
  });
  slide.addText("Graph conversion and CPU result-management are prominent costs beside the backend.", {
    x: 0.72, y: 6.52, w: 8.5, h: 0.24, margin: 0, fontSize: 16, bold: true, color: C.coral, fit: "shrink",
  });
  addFooter(slide, 4);
  slide.addNotes("About 70 seconds. This is the detailed optimize decomposition. The stacked bar uses percent of optimize, while the table uses percent of the full graph API, so the denominators are explicit. Graph conversion is 114.246 milliseconds of CPU work creating the specialized representation. The CUDA backend is 137.542 milliseconds. On the way back, return and assignment, Values merge, and destruction of converted data are all CPU-side result-management work. Values merge plus destruction alone add almost 98 milliseconds. The main point is that conversion and object/state management are first-order costs beside the backend, not bookkeeping after the GPU call.");
}

// Slide 5
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "Inside the CUDA backend: 0.138 s", "Measured hierarchy within the specialized backend.");
  addBadge(slide, "Measured", 11.65, 0.48, C.teal);
  addStackedBar(slide, backendRows.map((row) => ({ ...row, pct: (row.value / 137.542 * 100).toFixed(1), short: `${(row.value / 137.542 * 100).toFixed(0)}%` })), 0.72, 1.52, 11.9, 0.56, 137.542, 1.0);
  addTable(slide, [
    ["Backend layer", "Time", "% of backend"],
    ["Setup", "67.544 ms", "49.11%"],
    ["Solve loop", "29.968 ms", "21.79%"],
    ["Download Values", "40.030 ms", "29.10%"],
  ], 0.72, 2.55, [3.2, 1.7, 1.8], { rowH: 0.46, fontSize: 13.5, headerFill: C.green });
  addPanel(slide, 7.75, 2.55, 4.75, 1.08, C.teal, C.paleTeal);
  slide.addText("Setup: projection host build 38.686 ms", { x: 8.0, y: 2.83, w: 4.2, h: 0.2, margin: 0, fontSize: 15, bold: true, color: C.teal, fit: "shrink" });
  slide.addText("The largest setup detail is CPU construction of the projection batch.", { x: 8.0, y: 3.16, w: 4.2, h: 0.15, margin: 0, fontSize: 12.5, color: C.ink, fit: "shrink" });
  addPanel(slide, 0.72, 4.75, 5.95, 1.37, C.green, C.paleGreen);
  slide.addText("Solve loop: 29.968 ms", { x: 0.98, y: 5.0, w: 2.7, h: 0.2, margin: 0, fontSize: 15, bold: true, color: C.green });
  slide.addText("Dense Schur 23.009 ms | Hessian / damping diagonal 2.979 ms | linearized error change 3.655 ms", {
    x: 0.98, y: 5.38, w: 5.2, h: 0.28, margin: 0, fontSize: 13.5, color: C.ink, fit: "shrink",
  });
  addPanel(slide, 6.95, 4.75, 5.55, 1.37, C.amber, C.paleAmber);
  slide.addText("Download Values: 40.030 ms", { x: 7.2, y: 5.0, w: 4.9, h: 0.2, margin: 0, fontSize: 15, bold: true, color: C.amber, fit: "shrink" });
  slide.addText("Raw D2H 0.615 ms | Values rebuild 15.537 ms | remaining host allocation, wrapper, destruction, and tail 23.878 ms", {
    x: 7.2, y: 5.35, w: 4.9, h: 0.35, margin: 0, fontSize: 12.6, color: C.ink, fit: "shrink",
  });
  addFooter(slide, 5);
  slide.addNotes("About 60 seconds. Inside the backend, setup is almost half of the time and download plus Values reconstruction is another twenty-nine percent. Setup is led by the 38.686 millisecond projection host build. The solve loop is only 29.968 milliseconds, with dense Schur at 23.009 milliseconds, the Hessian or damping diagonal at 2.979 milliseconds, and linearized error change at 3.655 milliseconds. In contrast, the raw D2H copy is only 0.615 milliseconds. Much of download time is therefore host allocation, wrapping, destruction, and Values rebuilding around the copy.");
}

// Slide 6
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "What the profile says", "Measured specialized path: representation conversion and state management dominate the current bottleneck.");
  addBadge(slide, "Measured", 11.65, 0.48, C.teal);
  const profileRows = [
    { label: "Graph conversion", value: 114.246, color: C.teal },
    { label: "Values merge", value: 67.464, color: C.amber },
    { label: "Converted-data destruction", value: 30.247, color: C.coral },
    { label: "Solve loop", value: 29.968, color: C.green },
    { label: "Pure H2D + D2H", value: 1.936, color: C.blue },
  ];
  addBarRows(slide, profileRows, 0.75, 1.48, 3.0, 5.25, 114.246);
  addTable(slide, [
    ["Measured component", "Time", "% of graph API"],
    ["Graph conversion", "114.246 ms", "26.80%"],
    ["Values merge", "67.464 ms", "15.83%"],
    ["Converted-data destruction", "30.247 ms", "7.10%"],
    ["Solve loop", "29.968 ms", "7.03%"],
    ["Pure H2D + D2H", "1.936 ms", "0.45%"],
  ], 0.75, 4.1, [4.0, 1.75, 1.75], { rowH: 0.4, fontSize: 12.3, headerFill: C.navy });
  addPanel(slide, 8.9, 1.45, 3.55, 3.35, C.coral, C.paleCoral);
  slide.addText("Core conclusion", { x: 9.2, y: 1.8, w: 2.95, h: 0.22, margin: 0, fontSize: 16, bold: true, color: C.coral, align: "center" });
  slide.addText("The present bottleneck is CPU-side representation conversion and object/state management, not PCIe copies or the dense-Schur solve.", {
    x: 9.18, y: 2.35, w: 3.0, h: 1.15, margin: 0,
    fontSize: 18, bold: true, color: C.ink, align: "center", valign: "mid", fit: "shrink",
  });
  slide.addText("This conclusion applies to the specialized representation measured here.", {
    x: 9.2, y: 4.08, w: 2.95, h: 0.25, margin: 0,
    fontSize: 11.5, color: C.muted, align: "center", fit: "shrink",
  });
  addFooter(slide, 6);
  slide.addNotes("About 55 seconds. This comparison puts the relative scale in one place. Pure specialized H2D plus D2H is 1.936 milliseconds, or 0.45 percent of the 426.223 millisecond graph API. The dense-Schur solve loop is also smaller than graph conversion and Values merge. The careful conclusion is not that transfer never matters or that the GPU solve is free. It is that PCIe copies and dense Schur are not the dominant costs in this measured specialized representation. That motivates testing a different architecture rather than only tuning the kernel.");
}

// Slide 7
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "General-solver baseline", "Proposed Ceres-style separation, not an exact description of every Ceres path.");
  addBadge(slide, "Proposed", 11.65, 0.48, C.amber);
  addPipeline(slide, [
    { title: "CPU / TBB factor evaluation + linearization", note: "Arbitrary GTSAM factors stay on CPU.", color: C.teal, fill: C.paleTeal },
    { title: "Reusable sparse numeric J / H buffers", note: "Cache structure; refresh numeric values.", color: C.blue },
    { title: "H2D", note: "Upload generic linear-system numeric data.", color: C.amber, fill: C.paleAmber },
    { title: "GPU linear solve", note: "Normal equations plus cuDSS or dense solver.", color: C.green, fill: C.paleGreen },
    { title: "D2H delta", note: "Return only the solved update.", color: C.amber, fill: C.paleAmber },
    { title: "CPU Values retract + trial decision", note: "Manifold retract, error, and trust-region decision.", color: C.coral, fill: C.paleCoral },
  ], 0.55, 1.54, 12.23, 2.16);
  addPanel(slide, 0.72, 4.42, 5.7, 1.45, C.navy, "F1F4F8");
  slide.addText("Why this is general", { x: 1.0, y: 4.72, w: 5.1, h: 0.22, margin: 0, fontSize: 17, bold: true, color: C.navy, align: "center" });
  slide.addText("Arbitrary GTSAM factors remain in the CPU nonlinear layer. The GPU receives a generic linear system rather than an SFM-specific graph representation.", {
    x: 1.0, y: 5.1, w: 5.1, h: 0.48, margin: 0, fontSize: 14.5, color: C.ink, align: "center", valign: "mid", fit: "shrink",
  });
  addPanel(slide, 6.75, 4.42, 5.75, 1.45, C.amber, C.paleAmber);
  slide.addText("What remains a hypothesis", { x: 7.05, y: 4.72, w: 5.15, h: 0.22, margin: 0, fontSize: 17, bold: true, color: C.amber, align: "center" });
  slide.addText("Sparse packing, repeated numeric upload, delta mapping, and end-to-end convergence must be measured in the actual prototype.", {
    x: 7.05, y: 5.1, w: 5.15, h: 0.48, margin: 0, fontSize: 14.5, color: C.ink, align: "center", valign: "mid", fit: "shrink",
  });
  slide.addText([
    { text: "Sources: " },
    { text: "Ceres installation", options: { hyperlink: { url: "https://ceres-solver.readthedocs.io/latest/installation.html" }, color: C.blue, underline: true } },
    { text: " | " },
    { text: "Ceres nonlinear least-squares", options: { hyperlink: { url: "https://ceres-solver.readthedocs.io/latest/nnls_solving.html" }, color: C.blue, underline: true } },
  ], {
    x: 0.72, y: 6.55, w: 11.8, h: 0.17, margin: 0, fontSize: 10.5, color: C.muted, fit: "shrink",
  });
  addFooter(slide, 7);
  slide.addNotes("About 90 seconds. This is a proposed baseline architecture. I call it Ceres-style separation because the useful idea is to separate nonlinear residual and Jacobian evaluation from a selectable linear-algebra backend. I am not claiming that every Ceres path has this exact data flow. In this baseline, arbitrary GTSAM factors stay in the CPU and TBB nonlinear layer. The CPU fills reusable sparse numeric J or H buffers, the GPU receives the generic linear system, and only the solved delta returns. The CPU then performs manifold retraction, trial error, and the trust-region decision. This removes the current SFM-only graph conversion, while leaving the real costs of sparse packing and repeated upload to be measured.");
}

// Slide 8
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "Baseline feasibility timing", "Separately timed, synthetic per-candidate lower-bound arithmetic. Repeat-0-excluded sensitivity view.");
  addBadge(slide, "Measured", 10.35, 0.48, C.teal);
  addBadge(slide, "Estimated", 11.45, 0.48, C.amber);
  addTable(slide, [
    ["Measured CPU operation", "Mean", "Role"],
    ["CPU / TBB linearize", "28.316 ms", "Factor evaluation and linearization"],
    ["Values retract", "14.332 ms", "Apply a prebuilt compatible delta"],
    ["Trial graph.error", "11.440 ms", "Evaluate trial state"],
    ["CPU subtotal", "54.088 ms", "Illustrative arithmetic, separately timed"],
  ], 0.72, 1.46, [3.7, 1.7, 3.25], { rowH: 0.45, fontSize: 13, headerFill: C.teal, alignments: ["left", "right", "left"] });
  addPanel(slide, 9.5, 1.46, 2.98, 2.2, C.green, C.paleGreen);
  slide.addText("Synthetic GPU reference", { x: 9.75, y: 1.78, w: 2.48, h: 0.2, margin: 0, fontSize: 14, bold: true, color: C.green, align: "center", fit: "shrink" });
  slide.addText("8.051 ms", { x: 9.75, y: 2.18, w: 2.48, h: 0.34, margin: 0, fontSize: 26, bold: true, color: C.green, align: "center", fit: "shrink" });
  slide.addText("Hessian diagonal + dense Schur combined - projection", {
    x: 9.75, y: 2.70, w: 2.48, h: 0.52, margin: 0, fontSize: 12.2, color: C.ink, align: "center", fit: "shrink",
  });
  addCallout(slide, "Synthetic lower bound", "62.139 ms", "54.088 ms CPU + 8.051 ms GPU", 0.72, 4.15, 3.7, C.green);
  addCallout(slide, "Estimated generic transfer", "+11.283 ms", "115.1 MB / 9.50 GiB/s", 4.75, 4.15, 3.55, C.amber);
  addCallout(slide, "Lower bound + estimate", "73.422 ms", "still not end-to-end", 8.63, 4.15, 3.7, C.coral);
  addPanel(slide, 0.72, 5.63, 11.6, 0.72, C.coral, C.paleCoral);
  slide.addText("Caveat: not end-to-end. Sparse packing/layout, repeated upload, delta mapping, and convergence remain unmeasured. Do not compare these per-candidate synthetic figures directly to the 426.223 ms three-iteration API total.", {
    x: 0.98, y: 5.86, w: 11.05, h: 0.22, margin: 0, fontSize: 13.2, bold: true, color: C.ink, align: "center", fit: "shrink",
  });
  addFooter(slide, 8, "Hybrid feasibility | Dubrovnik 135");
  slide.addNotes("About 90 seconds. These numbers are deliberately separate from the current API profile. Each CPU operation was timed in its own loop, and the GPU reference is synthetic arithmetic from separately timed components. Excluding repeat zero gives 54.088 milliseconds for the illustrative CPU subtotal and 8.051 milliseconds for the synthetic GPU reference, or 62.139 milliseconds. The 115.1 MB generic Jacobian-transfer estimate is 11.283109 milliseconds before rounding, displayed as 11.283 milliseconds. Adding the unrounded estimate gives the displayed 73.422 millisecond combination. Neither number is an end-to-end runtime, and neither should be visually compared against the three-iteration 426.223 millisecond API total. Sparse packing, repeated upload, delta mapping, and convergence still need the actual prototype measurement.");
}

// Slide 9
{
  const slide = pptx.addSlide();
  addBg(slide, true);
  addBadge(slide, "Assessment", 10.65, 0.47, C.teal);
  addBadge(slide, "Proposed", 12.12, 0.47, C.amber);
  addTitle(slide, "Current assessment and next work", "Measured evidence justifies a hybrid baseline experiment, not a performance claim.", true);
  addPanel(slide, 0.72, 1.55, 5.7, 2.7, C.teal, "263956");
  slide.addText("Current assessment", { x: 1.0, y: 1.9, w: 5.1, h: 0.24, margin: 0, fontSize: 19, bold: true, color: C.teal, align: "center" });
  slide.addText("Separately timed CPU nonlinear subtotal is tens of milliseconds per candidate versus the prior 0.426 s three-iteration API run. Scopes differ, so this supports feasibility, not a speedup claim. Expected benefit: generality and removal of specialized graph conversion.", {
    x: 1.03, y: 2.32, w: 5.05, h: 1.22, margin: 0, fontSize: 16, bold: true, color: C.white, align: "center", valign: "mid", fit: "shrink",
  });
  slide.addText("Measured conclusion", { x: 1.03, y: 3.75, w: 5.05, h: 0.17, margin: 0, fontSize: 12, color: "D9E5EA", align: "center" });
  slide.addText("Next work", { x: 6.95, y: 1.7, w: 4.8, h: 0.25, margin: 0, fontSize: 19, bold: true, color: C.white });
  const nextSteps = [
    "Cache sparse structure.",
    "Direct TBB numeric fill.",
    "Upload only numeric values.",
    "GPU normal equations + cuDSS/dense solver.",
    "Download delta, then CPU retract/error.",
    "Measure full LM convergence.",
  ];
  nextSteps.forEach((step, index) => {
    const y = 2.18 + index * 0.62;
    slide.addShape(pptx.ShapeType.rect, { x: 6.98, y: y + 0.08, w: 0.17, h: 0.17, fill: { color: index < 3 ? C.teal : C.amber }, line: { color: index < 3 ? C.teal : C.amber } });
    slide.addText(step, { x: 7.35, y, w: 4.9, h: 0.27, margin: 0, fontSize: 16, bold: true, color: C.white, fit: "shrink" });
  });
  slide.addText("Success criterion: a measured end-to-end LM comparison with the same problem, convergence policy, and accounting scope.", {
    x: 6.98, y: 6.28, w: 5.35, h: 0.3, margin: 0, fontSize: 13, color: "D9E5EA", fit: "shrink",
  });
  slide.addNotes("About 60 seconds. The measured evidence says the CPU nonlinear pieces are sufficiently small to be worth implementing as a hybrid baseline. The separately timed CPU subtotal is on the order of tens of milliseconds per candidate, while the prior graph API measurement is 0.426 seconds for three iterations. Those scopes differ, so this supports feasibility rather than a speedup claim. The expected immediate benefit is generality and removal of specialized graph conversion. The next experiment is concrete: cache sparse structure, fill numeric buffers directly with TBB, upload only numeric values, solve on the GPU, then perform the actual CPU retract and error path in a full converging LM measurement.");
}

// Slide 10
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "Appendix: full dense-Schur breakdown", "Measured mean, Dubrovnik 135, warm-up excluded, 3 runs. Times in milliseconds.");
  addBadge(slide, "Measured", 11.65, 0.48, C.teal);
  addTable(slide, [
    ["Level", "Component", "Time", "% graph API", "% parent"],
    ["Graph API", "Optimizer construction", "36.203", "8.49%", "-"],
    ["Graph API", "optimize()", "376.192", "88.26%", "-"],
    ["Graph API", "Result/error queries", "13.828", "3.24%", "-"],
    ["optimize", "Graph conversion", "114.246", "26.80%", "30.37%"],
    ["optimize", "CUDA backend", "137.542", "32.27%", "36.56%"],
    ["optimize", "Backend return / assignment", "26.685", "6.26%", "7.09%"],
    ["optimize", "Values merge / state update", "67.464", "15.83%", "17.93%"],
    ["optimize", "Converted-data destruction", "30.247", "7.10%", "8.04%"],
    ["backend", "Setup", "67.544", "15.85%", "49.11%"],
    ["backend", "Solve loop", "29.968", "7.03%", "21.79%"],
    ["backend", "Download Values", "40.030", "9.39%", "29.10%"],
  ], 0.55, 1.35, [1.25, 4.65, 1.35, 1.55, 1.45], { rowH: 0.30, fontSize: 10.5, headerFill: C.navy });
  addTable(slide, [
    ["Detail", "Time", "Detail", "Time"],
    ["Setup: projection host build", "38.686 ms", "Solve: dense Schur", "23.009 ms"],
    ["Setup: pack values", "13.688 ms", "Solve: damping diagonal", "2.979 ms"],
    ["Setup: allocate trial values", "11.769 ms", "Solve: linearized error change", "3.655 ms"],
    ["Transfer: total H2D memcpy", "1.526 ms", "Transfer: total D2H memcpy", "0.615 ms"],
    ["Download: Values rebuild", "15.537 ms", "Pure H2D + D2H", "2.141 ms"],
  ], 0.55, 5.12, [3.85, 1.55, 3.85, 1.55], { rowH: 0.30, fontSize: 10.4, headerFill: C.teal });
  slide.addText("Note: the 1.936 ms specialized pure-copy figure cited in the feasibility summary comes from a separate transfer-focused run; this appendix uses the dense-Schur breakdown above.", {
    x: 0.55, y: 6.98, w: 11.8, h: 0.12, margin: 0, fontSize: 8.5, color: C.muted, fit: "shrink",
  });
  addFooter(slide, "A", "Appendix | dense-Schur breakdown");
  slide.addNotes("Appendix only. Use this table if asked for the timer hierarchy. It retains the top-level graph API, optimize, backend, setup, solve, and transfer rows. The final note distinguishes the dense-Schur run's transfer rows from the separate specialized transfer-focused timing cited in the feasibility summary.");
}

// Slide 11
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "Appendix: feasibility ranges and caveats", "Measured operations are separate loops. Derived hybrid values are sensitivity views, not sequential candidate calls.");
  addBadge(slide, "Measured", 10.35, 0.48, C.teal);
  addBadge(slide, "Estimated", 11.45, 0.48, C.amber);
  addTable(slide, [
    ["Operation", "All 30 mean [min, max] / CV", "Repeat-0-excluded 27 mean [min, max] / CV"],
    ["CPU / TBB linearize", "30.851 [25.119, 57.299] / 26.39%", "28.316 [25.119, 38.918] / 9.73%"],
    ["Values retract", "14.260 [12.320, 18.152] / 8.79%", "14.332 [12.591, 18.152] / 8.71%"],
    ["Trial graph.error", "11.644 [8.518, 16.235] / 20.00%", "11.440 [8.518, 14.845] / 19.26%"],
    ["GPU projection", "0.384 [0.379, 0.391] / 0.79%", "0.384 [0.379, 0.391] / 0.82%"],
    ["GPU Hessian diagonal", "0.919 [0.909, 0.924] / 0.39%", "0.919 [0.909, 0.924] / 0.41%"],
    ["GPU dense Schur combined", "7.513 [7.468, 7.576] / 0.42%", "7.516 [7.468, 7.576] / 0.43%"],
  ], 0.5, 1.32, [2.75, 4.65, 4.85], { rowH: 0.38, fontSize: 10.7, headerFill: C.navy, alignments: ["left", "center", "center"] });
  addPanel(slide, 0.5, 4.55, 5.85, 1.55, C.amber, C.paleAmber);
  slide.addText("First-index sensitivity", { x: 0.8, y: 4.83, w: 5.25, h: 0.2, margin: 0, fontSize: 15, bold: true, color: C.amber, align: "center" });
  slide.addText("Index-aligned CPU subtotal: repeat 0 mean 80.752 ms, repeat 1 mean 56.881 ms, repeat 2 mean 57.024 ms. Repeat 0 is materially slower.", {
    x: 0.8, y: 5.22, w: 5.25, h: 0.38, margin: 0, fontSize: 12.2, color: C.ink, align: "center", fit: "shrink",
  });
  addPanel(slide, 6.65, 4.55, 6.18, 1.55, C.teal, C.paleTeal);
  slide.addText("Exact transfer formula and provenance", { x: 6.95, y: 4.83, w: 5.58, h: 0.2, margin: 0, fontSize: 15, bold: true, color: C.teal, align: "center" });
  slide.addText("553,336 x (24 Jacobian + 2 residual) x 8 B = 115,093,888 B = 115.1 MB. / 9.50 GiB/s = 11.283 ms. Dataset: dubrovnik-135-90642-pre.txt; AMD EPYC Milan, NVIDIA A100 80 GB PCIe; CUDA, driver, and TBB thread configuration not recorded.", {
    x: 6.95, y: 5.16, w: 5.58, h: 0.55, margin: 0, fontSize: 11.2, color: C.ink, align: "center", fit: "shrink",
  });
  slide.addText("Scope caveats: CPU loops and GPU references are independently timed; the synthetic arithmetic does not measure sparse packing/layout, repeated upload, delta download/mapping, or LM convergence. All-sample lower bound: 64.803 ms. Repeat-0-excluded lower bound: 62.139 ms. With estimated transfer: 73.422 ms.", {
    x: 0.5, y: 6.42, w: 12.2, h: 0.35, margin: 0, fontSize: 10.8, bold: true, color: C.coral, align: "center", fit: "shrink",
  });
  addFooter(slide, "B", "Appendix | hybrid feasibility provenance");
  slide.addNotes("Appendix only. This provides the complete sensitivity view and provenance. The key observation is the slower first measured index for CPU linearization-related arithmetic. The exact generic-transfer formula is included because it is an estimate, not a measured H2D copy. The remaining caveats explain why the derived lower bounds are not end-to-end candidate timings.");
}

fs.mkdirSync(OUTPUT_DIR, { recursive: true });
pptx.writeFile({ fileName: OUTPUT_FILE });
