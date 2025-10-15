const map = L.map("map").setView([54.0, 18.6], 8); 

// Warstwa OpenStreetMap 
L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
  attribution: "&copy; OpenStreetMap contributors",
}).addTo(map);