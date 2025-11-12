// Dummy boats
const boats = [
  { name: "Alicja", lat: 53.9, lon: 21.7, status: "At sea", wind: "12kn"},
  { name: "Igła", lat: 53.8, lon: 21.55, status: "Anchored", wind: "9kn"},
  { name: "Rybak", lat: 53.7, lon: 21.65, status: "Docked", wind: "5kn"},
];

// Map init
const map = L.map("map").setView([53.8, 21.7], 10);

// OpenStreetMap + OpenSeaMap overlay
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 18 }).addTo(map);
L.tileLayer('https://tiles.openseamap.org/seamark/{z}/{x}/{y}.png').addTo(map);

// Marker cluster
const markerCluster = L.markerClusterGroup();

// Custom icon
const boatIcon = L.icon({
  iconUrl: "https://cdn-icons-png.flaticon.com/512/77/77521.png",
  iconSize: [32, 32],
  iconAnchor: [16, 16]
});

const markers = {}; // store markers by name

// Sidebar list
const list = document.getElementById("boat-list");
boats.forEach(boat => {
  const li = document.createElement("li");
  li.className = "boat-item";
  li.innerHTML = `${boat.name} <span>+</span>`;

  const info = document.createElement("div");
  info.className = "boat-details";
  info.innerHTML = `
    Status: ${boat.status}<br>
    Wind: ${boat.wind}<br>
    Position: ${boat.lat.toFixed(2)}, ${boat.lon.toFixed(2)}
  `;

  li.addEventListener("click", () => {
    const open = info.classList.toggle("open");
    li.querySelector("span").textContent = open ? "-" : "+";

    //  center map & open popup
    map.flyTo([boat.lat, boat.lon], 13);
    markers[boat.name].openPopup();
  });

  list.appendChild(li);
  list.appendChild(info);
});

// Markers
boats.forEach(b => {
  const marker = L.marker([b.lat, b.lon], { 
      icon: boatIcon,
      zIndexOffset: 500   //  prevents icon jumping
    })
    .bindPopup(`
      <b>${b.name}</b><br>
      Status: ${b.status}<br>
      Wind: ${b.wind}<br>
    `);

  markers[b.name] = marker; // reference

  markerCluster.addLayer(marker);
});

map.addLayer(markerCluster);

// Search
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


// Autocomplete / dynamiczny search
const searchInput = document.getElementById("search-input");
searchInput.addEventListener("input", () => {
  const query = searchInput.value.trim().toLowerCase();
  const filtered = boats.filter(b => b.name.toLowerCase().includes(query));
  renderBoatList(filtered);
});