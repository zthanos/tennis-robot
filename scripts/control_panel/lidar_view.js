/* Source-agnostic LaserScan diagnostics helpers (browser + Node tests). */
(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.LidarView = api;
})(typeof window !== "undefined" ? window : globalThis, () => {
  "use strict";

  function finite(value) {
    return Number.isFinite(value) ? value : null;
  }

  function derive(sensor, nowMs = Date.now(), staleAfterS = 2.5) {
    const ranges = Array.isArray(sensor?.ranges_m) ? sensor.ranges_m : [];
    const rangeMin = finite(sensor?.range_min_m ?? sensor?.min_range_m);
    const rangeMax = finite(sensor?.range_max_m ?? sensor?.max_range_m);
    const validRanges = ranges.filter(value => Number.isFinite(value)
      && (rangeMin === null || value >= rangeMin)
      && (rangeMax === null || value <= rangeMax));
    const receivedAtS = finite(sensor?.last_message_at_s);
    const ageS = receivedAtS === null ? null : Math.max(0, nowMs / 1000 - receivedAtS);
    const state = !ranges.length
      ? "waiting"
      : (ageS !== null && ageS <= staleAfterS ? "live" : "stale");
    return {
      state,
      age_s: ageS,
      frame_id: String(sensor?.frame_id || "unknown"),
      scan_rate_hz: finite(sensor?.scan_rate_hz),
      sample_count: Number.isFinite(sensor?.sample_count) ? sensor.sample_count : ranges.length,
      valid_count: Number.isFinite(sensor?.valid_sample_count) ? sensor.valid_sample_count : validRanges.length,
      invalid_count: Number.isFinite(sensor?.invalid_sample_count) ? sensor.invalid_sample_count : ranges.length - validRanges.length,
      angle_min_rad: finite(sensor?.angle_min_rad),
      angle_max_rad: finite(sensor?.angle_max_rad),
      angle_increment_rad: finite(sensor?.angle_increment_rad),
      range_min_m: rangeMin,
      range_max_m: rangeMax,
      scan_time_s: finite(sensor?.scan_time_s),
      nearest_m: validRanges.length ? Math.min(...validRanges) : null,
    };
  }

  return { derive };
});
