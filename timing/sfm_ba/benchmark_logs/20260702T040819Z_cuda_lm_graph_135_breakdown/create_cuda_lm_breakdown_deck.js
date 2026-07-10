const pptxgen = require("pptxgenjs");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "cu-gtsam benchmark";
pptx.company = "GTSAM CUDA SFM";
pptx.subject = "CUDA LM graph API timing breakdown";
pptx.title = "CUDA LM Graph API Timing Breakdown";
pptx.lang = "en-US";
pptx.theme = {
  headFontFace: "Aptos Display",
  bodyFontFace: "Aptos",
  lang: "en-US",
};
pptx.defineLayout({ name: "WIDE", width: 13.333, height: 7.5 });
pptx.layout = "WIDE";

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
  violet: "6E5AA8",
  blue: "3E74B8",
  white: "FFFFFF",
};

const W = 13.333;
const H = 7.5;
const M = 0.55;

function addBg(slide, dark = false) {
  slide.background = { color: dark ? C.navy : C.bg };
  if (!dark) {
    slide.addShape(pptx.ShapeType.rect, {
      x: 0,
      y: 0,
      w: W,
      h: 0.18,
      fill: { color: C.teal },
      line: { color: C.teal },
    });
  }
}

function addTitle(slide, title, subtitle) {
  slide.addText(title, {
    x: M,
    y: 0.34,
    w: 10.4,
    h: 0.45,
    margin: 0,
    fontFace: "Aptos Display",
    fontSize: 25,
    bold: true,
    color: C.ink,
    fit: "shrink",
  });
  if (subtitle) {
    slide.addText(subtitle, {
      x: M,
      y: 0.84,
      w: 10.8,
      h: 0.26,
      margin: 0,
      fontSize: 9.5,
      color: C.muted,
      fit: "shrink",
    });
  }
}

function addFooter(slide, idx) {
  slide.addText(`CUDA LM graph API timing | dubrovnik-135 | slide ${idx}`, {
    x: M,
    y: 7.14,
    w: 7,
    h: 0.18,
    margin: 0,
    fontSize: 7.5,
    color: "7B8490",
  });
}

function addBadge(slide, text, x, y, color) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x,
    y,
    w: 1.45,
    h: 0.34,
    rectRadius: 0.05,
    fill: { color },
    line: { color },
  });
  slide.addText(text, {
    x: x + 0.08,
    y: y + 0.085,
    w: 1.29,
    h: 0.14,
    margin: 0,
    align: "center",
    fontSize: 7.8,
    bold: true,
    color: C.white,
    fit: "shrink",
  });
}

function addCallout(slide, label, value, note, x, y, w, color) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x,
    y,
    w,
    h: 1.1,
    rectRadius: 0.05,
    fill: { color: C.white },
    line: { color: C.faint, width: 1 },
    shadow: { type: "outer", color: "000000", opacity: 0.08, blur: 1, angle: 45, offset: 1 },
  });
  slide.addText(value, {
    x: x + 0.18,
    y: y + 0.15,
    w: w - 0.36,
    h: 0.42,
    margin: 0,
    fontSize: 23,
    bold: true,
    color,
    fit: "shrink",
  });
  slide.addText(label, {
    x: x + 0.18,
    y: y + 0.59,
    w: w - 0.36,
    h: 0.22,
    margin: 0,
    fontSize: 9.5,
    bold: true,
    color: C.ink,
    fit: "shrink",
  });
  slide.addText(note, {
    x: x + 0.18,
    y: y + 0.82,
    w: w - 0.36,
    h: 0.18,
    margin: 0,
    fontSize: 7.8,
    color: C.muted,
    fit: "shrink",
  });
}

function addStackedBar(slide, items, x, y, w, h, total) {
  let cx = x;
  items.forEach((it) => {
    const bw = w * (it.time / total);
    slide.addShape(pptx.ShapeType.rect, {
      x: cx,
      y,
      w: Math.max(bw, 0.02),
      h,
      fill: { color: it.color },
      line: { color: it.color },
    });
    if (bw > 0.7) {
      slide.addText(`${it.pct.toFixed(1)}%`, {
        x: cx + 0.04,
        y: y + 0.12,
        w: bw - 0.08,
        h: 0.15,
        margin: 0,
        align: "center",
        fontSize: 8,
        bold: true,
        color: C.white,
        fit: "shrink",
      });
    }
    cx += bw;
  });
}

function addLegend(slide, items, x, y, columns = 2) {
  const rowH = 0.28;
  const colW = 2.7;
  items.forEach((it, i) => {
    const col = i % columns;
    const row = Math.floor(i / columns);
    const lx = x + col * colW;
    const ly = y + row * rowH;
    slide.addShape(pptx.ShapeType.rect, {
      x: lx,
      y: ly + 0.04,
      w: 0.14,
      h: 0.14,
      fill: { color: it.color },
      line: { color: it.color },
    });
    slide.addText(it.label, {
      x: lx + 0.2,
      y: ly,
      w: colW - 0.22,
      h: 0.2,
      margin: 0,
      fontSize: 7.6,
      color: C.muted,
      fit: "shrink",
    });
  });
}

function addTable(slide, rows, x, y, w, colWidths, opts = {}) {
  const headerFill = opts.headerFill || C.navy;
  const rowH = opts.rowH || 0.34;
  const fontSize = opts.fontSize || 8.4;
  rows.forEach((row, r) => {
    let cx = x;
    const fill = r === 0 ? headerFill : r % 2 ? C.white : "F0F4F7";
    const color = r === 0 ? C.white : C.ink;
    row.forEach((cell, c) => {
      slide.addShape(pptx.ShapeType.rect, {
        x: cx,
        y: y + r * rowH,
        w: colWidths[c],
        h: rowH,
        fill: { color: fill },
        line: { color: C.bg, width: 0.5 },
      });
      slide.addText(String(cell), {
        x: cx + 0.06,
        y: y + r * rowH + 0.095,
        w: colWidths[c] - 0.12,
        h: 0.13,
        margin: 0,
        fontSize,
        bold: r === 0 || c === 0,
        color,
        fit: "shrink",
        align: c === 0 ? "left" : "right",
      });
      cx += colWidths[c];
    });
  });
}

function addBars(slide, rows, x, y, w, rowH, maxVal, unit = "s", showValues = true) {
  rows.forEach((row, i) => {
    const yy = y + i * rowH;
    slide.addText(row.label, {
      x,
      y: yy + 0.045,
      w: 2.55,
      h: 0.14,
      margin: 0,
      fontSize: 8.5,
      bold: true,
      color: C.ink,
      fit: "shrink",
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: x + 2.8,
      y: yy + 0.035,
      w,
      h: 0.16,
      fill: { color: "E5EAF0" },
      line: { color: "E5EAF0" },
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: x + 2.8,
      y: yy + 0.035,
      w: w * row.value / maxVal,
      h: 0.16,
      fill: { color: row.color },
      line: { color: row.color },
    });
    if (showValues) {
      slide.addText(`${row.value.toFixed(6)} ${unit}`, {
        x: x + 2.8 + w + 0.14,
        y: yy + 0.03,
        w: 1.1,
        h: 0.14,
        margin: 0,
        fontSize: 8,
        color: C.muted,
        fit: "shrink",
      });
    }
  });
}

// Data from benchmark summaries.
const denseApi = 0.426223;
const denseTop = [
  { label: "Graph conversion", time: 0.114246, pct: 26.8, color: C.teal },
  { label: "CUDA backend", time: 0.137542, pct: 32.27, color: C.green },
  { label: "Value merge", time: 0.067464, pct: 15.83, color: C.amber },
  { label: "Return + destruction", time: 0.056932, pct: 13.36, color: C.coral },
  { label: "Construction + queries", time: 0.050031, pct: 11.73, color: C.violet },
];

const backendTotal = 0.137542;
const backendRows = [
  { label: "Setup", value: 0.067544, color: C.teal },
  { label: "Solve loop", value: 0.029968, color: C.green },
  { label: "Download Values", value: 0.040030, color: C.amber },
];
const loopRows = [
  { label: "Dense Schur solve", value: 0.023009, color: C.green },
  { label: "Linearized error change", value: 0.003655, color: C.blue },
  { label: "Damping diagonal", value: 0.002979, color: C.teal },
  { label: "Other loop work", value: 0.000325, color: C.muted },
];
const transferRows = [
  ["Pure copy", "Time", "Bytes", "% API"],
  ["H2D memcpy", "0.001566 s", "15.97 MB", "0.398%"],
  ["D2H memcpy", "0.000370 s", "2.19 MB", "0.094%"],
  ["H2D + D2H", "0.001936 s", "18.17 MB", "0.492%"],
];

// Slide 1
{
  const slide = pptx.addSlide();
  addBg(slide, true);
  slide.addText("CUDA LM Graph API", {
    x: 0.72,
    y: 0.62,
    w: 8.7,
    h: 0.65,
    margin: 0,
    fontFace: "Aptos Display",
    fontSize: 33,
    bold: true,
    color: C.white,
    fit: "shrink",
  });
  slide.addText("Detailed Timing Breakdown for Bundle Adjustment", {
    x: 0.76,
    y: 1.35,
    w: 8.8,
    h: 0.28,
    margin: 0,
    fontSize: 16,
    color: "D6E6EA",
    fit: "shrink",
  });
  addCallout(slide, "Graph API total", "0.426 s", "dense Schur, mean of 3", 0.78, 2.32, 2.65, C.teal);
  addCallout(slide, "Backend total", "0.138 s", "32.27% of API", 3.68, 2.32, 2.65, C.green);
  addCallout(slide, "Solve loop", "0.030 s", "7.03% of API", 6.58, 2.32, 2.65, C.amber);
  slide.addText("Dataset: dubrovnik-135-90642-pre   |   Warmup excluded   |   CUDA dense-Schur solver", {
    x: 0.8,
    y: 4.05,
    w: 8.9,
    h: 0.22,
    margin: 0,
    fontSize: 10,
    color: "D6E6EA",
    fit: "shrink",
  });
  slide.addText("Question: how much time is GPU math, and how much is API/data adaptation?", {
    x: 0.8,
    y: 4.45,
    w: 8.9,
    h: 0.26,
    margin: 0,
    fontSize: 14,
    bold: true,
    color: C.white,
    fit: "shrink",
  });
  slide.addShape(pptx.ShapeType.rect, { x: 10.35, y: 0, w: 2.98, h: 7.5, fill: { color: "0E7C86" }, line: { color: "0E7C86" } });
  const steps = ["GTSAM Graph", "Convert", "CUDA backend", "Download", "Merge Values"];
  steps.forEach((s, i) => {
    const yy = 1.0 + i * 1.05;
    slide.addShape(pptx.ShapeType.roundRect, { x: 10.75, y: yy, w: 1.95, h: 0.48, rectRadius: 0.04, fill: { color: i === 2 ? C.navy : C.white }, line: { color: C.white } });
    slide.addText(s, { x: 10.88, y: yy + 0.16, w: 1.7, h: 0.12, margin: 0, fontSize: 8.2, bold: true, color: i === 2 ? C.white : C.ink, align: "center", fit: "shrink" });
    if (i < steps.length - 1) slide.addShape(pptx.ShapeType.chevron, { x: 11.44, y: yy + 0.58, w: 0.55, h: 0.25, rotate: 90, fill: { color: C.white }, line: { color: C.white } });
  });
  slide.addNotes("Opening thesis: for dense Schur, the GPU solve is fast. End-to-end time is dominated by graph API adaptation and CPU-side object handling.");
}

// Slide 2
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "What Is Being Timed?", "The graph API starts from generic GTSAM objects, then adapts them to the CUDA SFM backend.");

  slide.addShape(pptx.ShapeType.roundRect, { x: 0.75, y: 1.28, w: 2.1, h: 0.72, rectRadius: 0.04, fill: { color: C.navy }, line: { color: C.navy } });
  slide.addText("NonlinearFactorGraph\n+ Values", { x: 0.92, y: 1.48, w: 1.76, h: 0.22, margin: 0, align: "center", fontSize: 10.2, bold: true, color: C.white, fit: "shrink" });

  const stages = [
    ["1", "Extract SFM subset", "Find Pose3/Point3 variables and projection factors"],
    ["2", "Build CUDA-shaped data", "Create SfmData, key maps, measurements, noise metadata"],
    ["3", "Run CUDA backend", "Pack/upload, solve LM, download packed values"],
    ["4", "Rebuild GTSAM Values", "Merge optimized SFM variables back into the original Values"],
  ];
  stages.forEach((s, i) => {
    const x = 3.25 + i * 2.28;
    slide.addShape(pptx.ShapeType.chevron, { x: x - 0.42, y: 1.48, w: 0.35, h: 0.28, fill: { color: "CBD5E1" }, line: { color: "CBD5E1" } });
    slide.addShape(pptx.ShapeType.roundRect, { x, y: 1.15, w: 1.86, h: 1.05, rectRadius: 0.04, fill: { color: i === 2 ? "EAF7EF" : C.white }, line: { color: i === 2 ? "BFE2CF" : C.faint } });
    slide.addShape(pptx.ShapeType.ellipse, { x: x + 0.13, y: 1.32, w: 0.32, h: 0.32, fill: { color: i === 0 ? C.teal : i === 1 ? C.blue : i === 2 ? C.green : C.amber }, line: { color: C.white, transparency: 100 } });
    slide.addText(s[0], { x: x + 0.13, y: 1.42, w: 0.32, h: 0.08, margin: 0, align: "center", fontSize: 7.5, bold: true, color: C.white });
    slide.addText(s[1], { x: x + 0.55, y: 1.26, w: 1.1, h: 0.2, margin: 0, fontSize: 8.7, bold: true, color: C.ink, fit: "shrink" });
    slide.addText(s[2], { x: x + 0.18, y: 1.68, w: 1.48, h: 0.28, margin: 0, align: "center", fontSize: 7.3, color: C.muted, fit: "shrink" });
  });

  slide.addText("Denominator for the breakdown", { x: 0.85, y: 2.78, w: 3.2, h: 0.18, margin: 0, fontSize: 11, bold: true, color: C.ink });
  slide.addText("Graph API total = optimizer construction + optimize() + result/error queries. In the 135 dense-Schur run, that total is 0.426223 s.", { x: 0.85, y: 3.12, w: 3.9, h: 0.46, margin: 0, fontSize: 10.5, color: C.muted, fit: "shrink" });
  slide.addShape(pptx.ShapeType.roundRect, { x: 0.85, y: 4.05, w: 3.85, h: 1.05, rectRadius: 0.05, fill: { color: "EAF6F7" }, line: { color: "C4E5E8" } });
  slide.addText("Important wording", { x: 1.08, y: 4.27, w: 3.35, h: 0.14, margin: 0, fontSize: 9.5, bold: true, color: C.teal });
  slide.addText("Here, graph conversion is not CUDA Graph capture. It means CPU-side conversion from generic GTSAM graph/Values into CUDA SFM arrays.", { x: 1.08, y: 4.58, w: 3.32, h: 0.34, margin: 0, fontSize: 9.6, color: C.ink, fit: "shrink" });

  addTable(slide, [
    ["Bucket", "Plain-English meaning"],
    ["Graph conversion", "Host scans generic graph and Values, extracts SFM-only data."],
    ["CUDA backend", "Packed CUDA setup, LM loop, and packed result download."],
    ["Value merge", "CPU rebuilds optimized GTSAM Values and updates original state."],
    ["Return/destruction", "C++ result copy/assignment and temporary object cleanup."],
  ], 5.35, 2.78, 6.7, [1.9, 4.15], { rowH: 0.36, fontSize: 8.25, headerFill: C.teal });
  slide.addText("Presenter framing: we are measuring an integration path, not only the GPU math kernel.", { x: 5.45, y: 5.17, w: 6.0, h: 0.28, margin: 0, fontSize: 14.5, bold: true, color: C.ink, fit: "shrink" });
  addFooter(slide, 2);
  slide.addNotes("Use this slide to align terminology. Say: the CUDA kernel may be fast, but the API path includes extraction, packing, object copies, and rebuilding Values.");
}

// Slide 3
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "Top-Level Breakdown: dense Schur", "Mean over 3 repeats; percentages are of CUDA LM graph API total.");
  addCallout(slide, "Graph API total", "0.426223 s", "presentation denominator", 0.72, 1.25, 2.45, C.navy);
  addCallout(slide, "CUDA backend", "32.27%", "0.137542 s", 3.42, 1.25, 2.45, C.green);
  addCallout(slide, "Solve loop", "7.03%", "0.029968 s", 6.12, 1.25, 2.45, C.amber);
  slide.addText("The backend is only about one third of graph API time.", { x: 8.95, y: 1.36, w: 3.55, h: 0.58, margin: 0, fontSize: 15, bold: true, color: C.ink, fit: "shrink" });
  addStackedBar(slide, denseTop, 0.72, 3.02, 11.8, 0.58, denseApi);
  addLegend(slide, denseTop, 0.75, 3.85, 3);
  addTable(slide, [
    ["Component", "Time", "%"],
    ["Graph conversion", "0.114246 s", "26.80"],
    ["CUDA backend total", "0.137542 s", "32.27"],
    ["Value merge/state", "0.067464 s", "15.83"],
    ["Return + destruction", "0.056932 s", "13.36"],
    ["Construction + queries", "0.050031 s", "11.73"],
  ], 0.9, 4.8, 7.3, [3.25, 1.7, 1.0], { rowH: 0.32, fontSize: 8.2 });
  slide.addShape(pptx.ShapeType.roundRect, { x: 8.85, y: 4.78, w: 3.55, h: 1.48, rectRadius: 0.05, fill: { color: "EAF6F7" }, line: { color: "C4E5E8" } });
  slide.addText("Key message", { x: 9.1, y: 5.0, w: 3.0, h: 0.16, margin: 0, fontSize: 10, bold: true, color: C.teal });
  slide.addText("The graph API wrapper, not the dense Schur math, is where most end-to-end time goes.", { x: 9.1, y: 5.27, w: 3.0, h: 0.55, margin: 0, fontSize: 12.5, bold: true, color: C.ink, fit: "shrink" });
  addFooter(slide, 3);
}

// Slide 4
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "Inside the CUDA Backend", "The backend itself is setup-heavy; the solve loop is small.");
  addBars(slide, backendRows, 0.82, 1.45, 4.6, 0.55, backendTotal, "s", false);
  addTable(slide, [
    ["Backend component", "Time", "% API", "% backend"],
    ["Setup before solve", "0.067544 s", "15.85", "49.11"],
    ["Solve loop", "0.029968 s", "7.03", "21.79"],
    ["Download Values", "0.040030 s", "9.39", "29.10"],
  ], 0.9, 3.55, 6.55, [2.65, 1.4, 1.1, 1.2], { rowH: 0.34, fontSize: 8.2 });
  slide.addShape(pptx.ShapeType.roundRect, { x: 8.55, y: 1.32, w: 3.65, h: 2.15, rectRadius: 0.05, fill: { color: C.white }, line: { color: C.faint } });
  slide.addText("Backend setup includes", { x: 8.8, y: 1.62, w: 3.15, h: 0.18, margin: 0, fontSize: 11, bold: true, color: C.ink });
  slide.addText("Pack values, allocate trial values, build projection batch, compute initial error, construct solver.", { x: 8.8, y: 1.98, w: 3.1, h: 0.42, margin: 0, fontSize: 11, color: C.muted, fit: "shrink" });
  slide.addText("Largest setup item: projection batch build, 0.0413 s.", { x: 8.8, y: 2.64, w: 3.1, h: 0.2, margin: 0, fontSize: 10.5, bold: true, color: C.teal, fit: "shrink" });
  slide.addText("Takeaway: optimizing only the solve loop has limited end-to-end upside unless setup/download are also reduced.", { x: 7.95, y: 4.58, w: 4.0, h: 0.6, margin: 0, fontSize: 14, bold: true, color: C.ink, fit: "shrink" });
  addFooter(slide, 4);
}

// Slide 5
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "Solve Loop Detail", "Once inside the LM loop, dense Schur dominates, but the whole loop is only 7.03% of graph API time.");
  addBars(slide, loopRows, 0.82, 1.35, 4.8, 0.56, 0.023009, "s", false);
  addTable(slide, [
    ["Solve-loop component", "Time", "% loop"],
    ["Dense Schur solve", "0.023009 s", "76.78"],
    ["Linearized error change", "0.003655 s", "12.20"],
    ["Damping diagonal", "0.002979 s", "9.94"],
    ["Everything else", "~0.000325 s", "~1.08"],
  ], 0.9, 3.85, 6.05, [2.95, 1.45, 1.0], { rowH: 0.34, fontSize: 8.3 });
  slide.addShape(pptx.ShapeType.roundRect, { x: 8.85, y: 1.28, w: 3.1, h: 1.65, rectRadius: 0.05, fill: { color: "EAF7EF" }, line: { color: "BFE2CF" } });
  slide.addText("7.03%", { x: 9.05, y: 1.62, w: 2.7, h: 0.38, margin: 0, align: "center", fontSize: 27, bold: true, color: C.green, fit: "shrink" });
  slide.addText("of graph API total", { x: 9.05, y: 2.1, w: 2.7, h: 0.16, margin: 0, align: "center", fontSize: 9.5, bold: true, color: C.ink });
  slide.addText("0.029968 s solve loop", { x: 9.05, y: 2.36, w: 2.7, h: 0.16, margin: 0, align: "center", fontSize: 8.8, color: C.muted });
  slide.addText("The math kernel is not the main end-to-end bottleneck in this graph API path.", { x: 8.05, y: 4.55, w: 3.7, h: 0.55, margin: 0, fontSize: 14, bold: true, color: C.ink, fit: "shrink" });
  addFooter(slide, 5);
}

// Slide 6
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "Is It Just GPU Transfer? No", "Separate dense-Schur transfer run; graph API total 0.393604 s.");
  addTable(slide, transferRows, 0.78, 1.35, 7.1, [2.15, 1.45, 1.45, 1.0], { rowH: 0.36, fontSize: 8.3, headerFill: C.teal });
  addCallout(slide, "Total raw copy", "0.001936 s", "H2D + D2H = 0.492% API", 8.65, 1.25, 2.8, C.teal);
  addCallout(slide, "Projection host build", "0.036242 s", "9.21% API", 8.65, 2.68, 2.8, C.coral);
  addCallout(slide, "Download Values rebuild", "0.015066 s", "3.83% API", 8.65, 4.11, 2.8, C.amber);
  slide.addText("Conclusion: raw memory copy is not the bottleneck. CPU packing/rebuilding is.", { x: 0.85, y: 5.9, w: 8.2, h: 0.28, margin: 0, fontSize: 15, bold: true, color: C.ink, fit: "shrink" });
  addFooter(slide, 6);
}

// Slide 7
{
  const slide = pptx.addSlide();
  addBg(slide);
  addTitle(slide, "Where the Old “Other” Bucket Went", "New instrumentation splits the residual into C++ result assignment and temporary data cleanup.");
  addTable(slide, [
    ["Component", "Time", "% API"],
    ["Backend return/assignment", "0.026685 s", "6.26"],
    ["Converted data destruction", "0.030247 s", "7.10"],
    ["Remaining residual", "0.000008 s", "~0.00"],
  ], 0.85, 1.38, 5.75, [2.8, 1.5, 1.0], { rowH: 0.38, fontSize: 8.5, headerFill: C.coral });
  slide.addShape(pptx.ShapeType.roundRect, { x: 7.1, y: 1.25, w: 4.65, h: 1.4, rectRadius: 0.05, fill: { color: C.white }, line: { color: C.faint } });
  slide.addText("Why assignment is slow", { x: 7.38, y: 1.55, w: 4.1, h: 0.18, margin: 0, fontSize: 11, bold: true, color: C.ink });
  slide.addText("result_ = OptimizeCudaSfmImpl(...) deep-copies Values optimizedValues because Values lacks move assignment.", { x: 7.38, y: 1.88, w: 4.1, h: 0.32, margin: 0, fontSize: 9.5, color: C.muted, fit: "shrink" });
  slide.addShape(pptx.ShapeType.roundRect, { x: 7.1, y: 3.0, w: 4.65, h: 1.55, rectRadius: 0.05, fill: { color: C.white }, line: { color: C.faint } });
  slide.addText("What gets destroyed", { x: 7.38, y: 3.29, w: 4.1, h: 0.18, margin: 0, fontSize: 11, bold: true, color: C.ink });
  slide.addText("Temporary CudaSfmFactorGraphData: SfmData cameras/tracks/measurements, key vectors, noise metadata.", { x: 7.38, y: 3.62, w: 4.05, h: 0.35, margin: 0, fontSize: 9.5, color: C.muted, fit: "shrink" });
  slide.addText("This is C++ object lifetime cost, not hidden GPU work.", { x: 0.9, y: 5.52, w: 8.3, h: 0.35, margin: 0, fontSize: 16, bold: true, color: C.coral, fit: "shrink" });
  addFooter(slide, 7);
}

// Slide 8
{
  const slide = pptx.addSlide();
  addBg(slide, true);
  slide.addText("Takeaways", { x: 0.78, y: 0.6, w: 5.0, h: 0.45, margin: 0, fontSize: 30, bold: true, color: C.white });
  const takeaways = [
    ["1", "GPU solve is already fast", "Dense-Schur solve loop is 7.03% of graph API time."],
    ["2", "API/data structures dominate", "Graph conversion, merge, object movement, setup, and download account for most time."],
    ["3", "Raw copies are not the bottleneck", "H2D + D2H memcpy is about 0.49% in the transfer-focused run."],
    ["4", "Next targets are host-side", "Move-optimize Values, reduce conversion/merge, and cache projection structure."],
  ];
  takeaways.forEach((t, i) => {
    const y = 1.55 + i * 1.1;
    slide.addShape(pptx.ShapeType.ellipse, { x: 0.95, y, w: 0.55, h: 0.55, fill: { color: i === 0 ? C.teal : i === 1 ? C.amber : i === 2 ? C.green : C.coral }, line: { color: C.white, transparency: 100 } });
    slide.addText(t[0], { x: 0.95, y: y + 0.17, w: 0.55, h: 0.12, margin: 0, align: "center", fontSize: 10, bold: true, color: C.white });
    slide.addText(t[1], { x: 1.75, y: y + 0.02, w: 4.3, h: 0.2, margin: 0, fontSize: 14, bold: true, color: C.white, fit: "shrink" });
    slide.addText(t[2], { x: 1.75, y: y + 0.34, w: 6.9, h: 0.18, margin: 0, fontSize: 9.5, color: "D6E6EA", fit: "shrink" });
  });
  slide.addShape(pptx.ShapeType.rect, { x: 9.55, y: 0, w: 3.78, h: 7.5, fill: { color: C.teal }, line: { color: C.teal } });
  slide.addText("Main message", { x: 10.0, y: 1.35, w: 2.85, h: 0.18, margin: 0, fontSize: 11, bold: true, color: C.white });
  slide.addText("The next speedups are around the solve, not inside the dense Schur kernel or PCIe copies.", { x: 10.0, y: 1.85, w: 2.75, h: 1.35, margin: 0, fontSize: 19, bold: true, color: C.white, fit: "shrink" });
  slide.addText("Proposed next experiment: add Values move assignment and measure return/assignment again.", { x: 10.0, y: 5.25, w: 2.75, h: 0.55, margin: 0, fontSize: 10.5, color: "E4F3F5", fit: "shrink" });
  slide.addNotes("Close by emphasizing this is a measurement-driven optimization plan. The surprising result is that raw copies are tiny and object/API overheads dominate.");
}

pptx.writeFile({
  fileName: "timing/sfm_ba/benchmark_logs/20260702T040819Z_cuda_lm_graph_135_breakdown/cuda_lm_graph_breakdown_presentation.pptx",
});
