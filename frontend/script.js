// Inicjalizacja mapy 
const map = L.map("map").setView([54.4, 18.6], 9);

// Warstwa bazowa
L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
  attribution: '&copy; OpenStreetMap contributors',
}).addTo(map);

// Warstwa morska 
L.tileLayer("https://tiles.openseamap.org/seamark/{z}/{x}/{y}.png", {
  attribution: '&copy; <a href="https://www.openseamap.org/">OpenSeaMap</a> contributors',
}).addTo(map);

// Przykładowe dane jachtów
const boats = [
  {
    id: 1,
    name: "Jacht_01",
    coords: [54.1, 18.8],
    status: "ok",
    speed: 4.2,
  },
  {
    id: 2,
    name: "Jacht_02",
    coords: [54.25, 18.55],
    status: "alert",
    speed: 8.1,
  },
  {
    id: 3,
    name: "Jacht_03",
    coords: [53.95, 18.7],
    status: "ok",
    speed: 3.7,
  },
];

// Dodaj jachty do mapy
boats.forEach((boat) => {
  const icon = L.divIcon({
    html: boat.status === "ok" ? "⛵" : "🚨",
    className: "boat-icon",
    iconSize: [30, 30],
  });

  L.marker(boat.coords, { icon })
    .addTo(map)
    .bindPopup(
      `<b>${boat.name}</b><br>
       Prędkość: ${boat.speed} węzłów<br>
       Status: <span class="status ${boat.status}">${boat.status}</span>`
    );
});

// Wyświetlenie listy jachtów w panelu bocznym
const boatList = document.getElementById("boat-list");

boats.forEach((boat) => {
  const li = document.createElement("li");
  li.classList.add("boat-item");
  li.innerHTML = `
    <strong>${boat.name}</strong><br>
    Prędkość: ${boat.speed} węzłów<br>
    Status: <span class="status ${boat.status}">${boat.status}</span>
  `;
  boatList.appendChild(li);
});