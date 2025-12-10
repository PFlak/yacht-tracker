// Map init
const map = L.map("map").setView([53.8, 21.7], 10);

// OpenStreetMap + OpenSeaMap overlay
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 18 }).addTo(map);
L.tileLayer('https://tiles.openseamap.org/seamark/{z}/{x}/{y}.png').addTo(map);

// Marker cluster
const markerCluster = L.markerClusterGroup();

// Icon
const boatIcon = L.icon({
  iconUrl: "https://cdn-icons-png.flaticon.com/512/77/77521.png",
  iconSize: [32, 32],
  iconAnchor: [16, 16]
});

const markers = {}; 

// Sidebar list
const list = document.getElementById("boat-list");
const myFleetBtn = document.querySelector("nav a[href='/my_fleet']");

// Ukryj przycisk My Fleet i log out dopóki nie wiemy, czy użytkownik zalogowany
myFleetBtn.style.display = "none";
const logoutBtn = document.querySelector("nav a.logout");
logoutBtn.style.display = "none"; // ukryj na start


// Funkcja do ładowania floty zalogowanego użytkownika
async function loadFleet() {
  try {
    const res = await fetch("/ui/recorders");
    if(res.status === 401){
      // niezalogowany
      list.innerHTML = "<li>Log in to show your fleet</li>";
      return;
    }

    const data = await res.json();
    const boats = data.array || [];

    if(boats.length > 0) myFleetBtn.style.display = "inline-block"; // pokaż My Fleet
    logoutBtn.style.display = "inline-block";  // ← pokaż logout


    // render sidebar
    list.innerHTML = "";
    boats.forEach(boat => {
      const li = document.createElement("li");
      li.className = "boat-item";
      li.innerHTML = `${boat.name} <span>+</span>`;

      const info = document.createElement("div");
      info.className = "boat-details";
      info.innerHTML = `
        Status: ${boat.is_online ? "Online" : "Offline"}<br>
        Type: ${boat.boat_type || "-"}<br>
        Model: ${boat.model || "-"}<br>
      `;

      li.addEventListener("click", () => {
        const open = info.classList.toggle("open");
        li.querySelector("span").textContent = open ? "-" : "+";

        // jeśli mamy pozycję gps w boat.last_seen_at itp., ustaw marker
        if(boat.latitude && boat.longitude){
          map.flyTo([boat.latitude, boat.longitude], 13);
          if(markers[boat.name]) markers[boat.name].openPopup();
        }
      });
      list.appendChild(li);
      list.appendChild(info);
      // marker na mapie (jeżeli mamy pozycję)
      if(boat.latitude && boat.longitude){
        const marker = L.marker([boat.latitude, boat.longitude], { icon: boatIcon })
          .bindPopup(`<b>${boat.name}</b><br>Status: ${boat.is_online ? "Online":"Offline"}<br>Type: ${boat.boat_type || "-"}<br>Model: ${boat.model || "-"}`);
        markers[boat.name] = marker;
        markerCluster.addLayer(marker);
      }
    });

    map.addLayer(markerCluster);
  } catch(err) {
    console.error("Błąd ładowania floty:", err);
    list.innerHTML = "<li>Unable to load fleet</li>";
  }
}

// Wyszukiwanie
document.getElementById("search-btn").addEventListener("click", () => {
  const q = document.getElementById("search-input").value.trim().toLowerCase();
  const found = boats.find(b => b.name.toLowerCase() === q);

  if (!found) {
    alert("Nie znaleziono łódki!");
    return;
  }

  map.flyTo([found.lat, found.lon], 13);
  markers[found.name].openPopup();
});
loadFleet();