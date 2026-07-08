window.ControlPanelSurveyHistory = (() => {
  function escHtml(value) {
    return String(value)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  async function load() {
    try {
      const response = await fetch("/api/surveys", { cache: "no-store" });
      const data = await response.json();
      render(data.surveys || []);
    } catch (_) {}
  }

  function render(surveys) {
    const tbody = document.getElementById("surveyRows");
    const empty = document.getElementById("surveyEmpty");
    const table = document.getElementById("surveyTable");
    if (!tbody || !empty || !table) return;
    if (!surveys.length) {
      table.style.display = "none";
      empty.style.display = "block";
      return;
    }
    table.style.display = "";
    empty.style.display = "none";
    tbody.innerHTML = surveys.map(survey => {
      const surveyedAt = survey.surveyed_at ? new Date(survey.surveyed_at * 1000).toLocaleString() : "-";
      const status = survey.status || "SUCCESS";
      const reasonTip = survey.failure_reason ? ` title="${escHtml(survey.failure_reason)}"` : "";
      const statusEl = status === "SUCCESS"
        ? `<span style="color:var(--accent);font-size:11px;font-weight:600;">SUCCESS</span>`
        : `<span style="color:var(--danger);font-size:11px;font-weight:600;cursor:help;"${reasonTip}>FAILED</span>`;
      const lengthM = survey.court_length_m != null ? survey.court_length_m.toFixed(2) : (
        survey.east_x != null && survey.west_x != null ? (survey.east_x - survey.west_x).toFixed(2) : "-");
      const widthM = survey.court_width_m != null ? survey.court_width_m.toFixed(2) : (
        survey.north_y != null && survey.south_y != null ? (survey.north_y - survey.south_y).toFixed(2) : "-");
      const ro = value => (value != null && Number.isFinite(value) ? value.toFixed(2) : "-");
      return `<tr>
        <td style="color:var(--muted);">${survey.id}</td>
        <td>${surveyedAt}</td>
        <td>${escHtml(survey.vendor_name || "-")}</td>
        <td>${escHtml(survey.court_name || "-")}</td>
        <td>${statusEl}</td>
        <td style="color:var(--muted);font-size:11px;">${escHtml(survey.survey_type || "-")}</td>
        <td style="color:var(--accent-2);font-weight:600;font-variant-numeric:tabular-nums;">${lengthM}</td>
        <td style="color:var(--accent-2);font-weight:600;font-variant-numeric:tabular-nums;">${widthM}</td>
        <td style="font-variant-numeric:tabular-nums;">${ro(survey.near_baseline_to_fence_m)}</td>
        <td style="font-variant-numeric:tabular-nums;">${ro(survey.far_baseline_to_fence_m)}</td>
        <td style="font-variant-numeric:tabular-nums;">${ro(survey.left_sideline_to_fence_m)}</td>
        <td style="font-variant-numeric:tabular-nums;">${ro(survey.right_sideline_to_fence_m)}</td>
        <td style="color:var(--muted);">${survey.obstacle_count ?? "-"}</td>
        <td style="color:var(--muted);">${survey.point_count ?? "-"}</td>
      </tr>`;
    }).join("");
  }

  return { load };
})();
