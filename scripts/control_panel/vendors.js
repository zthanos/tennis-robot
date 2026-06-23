window.ControlPanelVendors = (() => {
  let vendorData = { vendors: [], courts: [], active: {} };
  let onChange = () => {};

  function escHtml(value) {
    return String(value)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  function cloneData() {
    return JSON.parse(JSON.stringify(vendorData));
  }

  function getActive() {
    return vendorData.active || {};
  }

  function setOnChange(callback) {
    onChange = typeof callback === "function" ? callback : () => {};
  }

  async function load() {
    try {
      const response = await fetch("/api/vendors", { cache: "no-store" });
      vendorData = await response.json();
    } catch (_) {}
    render();
  }

  async function save(data) {
    try {
      await fetch("/api/vendors", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(data)
      });
      vendorData = data;
    } catch (_) {}
    render();
  }

  function updateCourtSelect(selectId, vendorId) {
    const select = document.getElementById(selectId);
    if (!select) return;
    const { courts, active } = vendorData;
    const list = vendorId ? courts.filter(court => court.vendor_id === vendorId) : courts;
    select.innerHTML = `<option value="">- select court -</option>` +
      list.map(court => `<option value="${escHtml(court.id)}"${court.id === (active && active.court_id) ? " selected" : ""}>${escHtml(court.name)}</option>`).join("");
  }

  function render() {
    const { vendors = [], courts = [], active = {} } = vendorData;

    const sidebarEl = document.getElementById("activeSessionSidebar");
    if (sidebarEl) {
      if (active && active.vendor_id) {
        const vendor = vendors.find(item => item.id === active.vendor_id);
        const court = courts.find(item => item.id === active.court_id);
        sidebarEl.textContent = `Session: ${vendor ? vendor.name : "?"} / ${court ? court.name : "?"}`;
        sidebarEl.style.color = "var(--accent)";
      } else {
        sidebarEl.textContent = "Session: none";
        sidebarEl.style.color = "var(--muted)";
      }
    }

    const displayEl = document.getElementById("activeSessionDisplay");
    if (displayEl) {
      if (active && active.vendor_id) {
        const vendor = vendors.find(item => item.id === active.vendor_id);
        const court = courts.find(item => item.id === active.court_id);
        displayEl.innerHTML = `
          <strong style="color:var(--accent);font-size:15px;">${escHtml(vendor ? vendor.name : "Unknown vendor")}</strong>
          <span style="color:var(--muted);margin:0 8px;">/</span>
          <span style="color:var(--ink);">${escHtml(court ? court.name : "Unknown court")}</span>
          ${court && court.surface ? `<span style="color:var(--muted);font-size:12px;margin-left:10px;">${escHtml(court.surface)}</span>` : ""}
          ${vendor && vendor.address ? `<div style="color:var(--muted);font-size:12px;margin-top:4px;">${escHtml(vendor.address)}</div>` : ""}
        `;
      } else {
        displayEl.innerHTML = `<span style="color:var(--muted);font-size:13px;">No active session. Select a vendor and court below.</span>`;
      }
    }

    ["activeVendorSelect", "courtVendorSelect"].forEach(id => {
      const select = document.getElementById(id);
      if (!select) return;
      const previous = select.value;
      select.innerHTML = `<option value="">- select vendor -</option>` +
        vendors.map(vendor => `<option value="${escHtml(vendor.id)}"${vendor.id === previous ? " selected" : ""}>${escHtml(vendor.name)}</option>`).join("");
    });

    const activeVendorSelect = document.getElementById("activeVendorSelect");
    if (activeVendorSelect && !activeVendorSelect.value && active && active.vendor_id) activeVendorSelect.value = active.vendor_id;
    updateCourtSelect("activeCourtSelect", activeVendorSelect ? activeVendorSelect.value : "");

    const vendorList = document.getElementById("vendorList");
    if (vendorList) {
      if (!vendors.length) {
        vendorList.innerHTML = `<div style="color:var(--muted);font-size:13px;padding:8px 0;">No vendors yet.</div>`;
      } else {
        vendorList.innerHTML = vendors.map(vendor => `
          <div style="display:flex;justify-content:space-between;align-items:flex-start;padding:10px 0;border-bottom:1px solid var(--line);">
            <div>
              <strong>${escHtml(vendor.name)}</strong>
              ${vendor.address ? `<div style="font-size:12px;color:var(--muted);margin-top:2px;">${escHtml(vendor.address)}</div>` : ""}
            </div>
            <button onclick="ControlPanelVendors.deleteVendor('${escHtml(vendor.id)}')" style="background:rgba(255,107,95,0.12);border:1px solid rgba(255,107,95,0.28);color:var(--danger);border-radius:5px;padding:5px 10px;cursor:pointer;font:inherit;font-size:12px;flex-shrink:0;margin-left:12px;">Delete</button>
          </div>
        `).join("");
      }
    }

    const courtList = document.getElementById("courtList");
    if (courtList) {
      if (!courts.length) {
        courtList.innerHTML = `<div style="color:var(--muted);font-size:13px;padding:8px 0;">No courts yet.</div>`;
      } else {
        courtList.innerHTML = courts.map(court => {
          const vendor = vendors.find(item => item.id === court.vendor_id);
          const isActive = active && active.court_id === court.id;
          return `
            <div style="display:flex;justify-content:space-between;align-items:flex-start;padding:10px ${isActive ? "8px" : "0"};border-bottom:1px solid var(--line);${isActive ? "background:rgba(47,208,143,0.05);border-radius:6px;" : ""}">
              <div>
                <strong>${escHtml(court.name)}</strong>${isActive ? ` <span style="font-size:11px;color:var(--accent);">active</span>` : ""}
                <div style="font-size:12px;color:var(--muted);margin-top:2px;">${escHtml(vendor ? vendor.name : "-")} / ${escHtml(court.surface || "-")}</div>
                ${court.notes ? `<div style="font-size:12px;color:var(--muted);">${escHtml(court.notes)}</div>` : ""}
              </div>
              <button onclick="ControlPanelVendors.deleteCourt('${escHtml(court.id)}')" style="background:rgba(255,107,95,0.12);border:1px solid rgba(255,107,95,0.28);color:var(--danger);border-radius:5px;padding:5px 10px;cursor:pointer;font:inherit;font-size:12px;flex-shrink:0;margin-left:12px;">Delete</button>
            </div>
          `;
        }).join("");
      }
    }

    onChange();
  }

  function deleteVendor(id) {
    const data = cloneData();
    data.vendors = data.vendors.filter(vendor => vendor.id !== id);
    data.courts = data.courts.filter(court => court.vendor_id !== id);
    if (data.active && data.active.vendor_id === id) data.active = {};
    save(data);
  }

  function deleteCourt(id) {
    const data = cloneData();
    data.courts = data.courts.filter(court => court.id !== id);
    if (data.active && data.active.court_id === id) data.active = Object.assign({}, data.active, { court_id: null });
    save(data);
  }

  function initView() {
    const activeVendorSelect = document.getElementById("activeVendorSelect");
    if (activeVendorSelect) activeVendorSelect.addEventListener("change", function () {
      updateCourtSelect("activeCourtSelect", this.value);
    });

    const activeSessionForm = document.getElementById("activeSessionForm");
    if (activeSessionForm) activeSessionForm.addEventListener("submit", function (event) {
      event.preventDefault();
      const vendorId = document.getElementById("activeVendorSelect").value;
      const courtId = document.getElementById("activeCourtSelect").value;
      if (!vendorId || !courtId) return;
      const data = cloneData();
      data.active = { vendor_id: vendorId, court_id: courtId };
      save(data);
    });

    const addVendorForm = document.getElementById("addVendorForm");
    if (addVendorForm) addVendorForm.addEventListener("submit", function (event) {
      event.preventDefault();
      const name = document.getElementById("vendorName").value.trim();
      const address = document.getElementById("vendorAddress").value.trim();
      if (!name) return;
      const data = cloneData();
      data.vendors.push({ id: "v" + Date.now(), name, address });
      save(data);
      this.reset();
    });

    const addCourtForm = document.getElementById("addCourtForm");
    if (addCourtForm) addCourtForm.addEventListener("submit", function (event) {
      event.preventDefault();
      const vendor_id = document.getElementById("courtVendorSelect").value;
      const name = document.getElementById("courtName").value.trim();
      const surface = document.getElementById("courtSurface").value;
      const notes = document.getElementById("courtNotes").value.trim();
      if (!vendor_id || !name) return;
      const data = cloneData();
      data.courts.push({ id: "c" + Date.now(), vendor_id, name, surface, notes });
      save(data);
      this.reset();
    });

    render();
  }

  return { getActive, setOnChange, load, initView, deleteVendor, deleteCourt };
})();
