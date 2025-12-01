// Lista łodzi 
let boats = [];

const boatList = document.getElementById("boat-list");
const addModal = document.getElementById("addBoatModal");
const deleteModal = document.getElementById("deleteBoatModal");
const deleteSelect = document.getElementById("deleteBoatSelect");

// Funkcja pobierająca łodzie z backendu
async function loadBoats() {
  try {
    const res = await fetch("/ui/fleet");
    const data = await res.json();
    boats = data.array || [];
    renderBoats();
  } catch (err) {
    console.error("Błąd ładowania łodzi:", err);
  }
}

// Render listy łódek
function renderBoats() {
  boatList.innerHTML = "";
  deleteSelect.innerHTML = "";

  boats.forEach(b => {
    const li = document.createElement("li");
    li.innerHTML = `${b.name} <span>+</span>`;

    const info = document.createElement("div");
    info.className = "boat-details";
    info.innerHTML = `
 Model: ${b.model || "brak"}<br>
      Type: ${b.boat_type || "brak"}<br>
      Contact: ${b.contact_number || "brak"}<br>
      <img src="https://source.unsplash.com/400x30${Math.floor(Math.random() * 9)}/?sail" width="200" style="margin-top:8px;border-radius:8px">
    `;

    li.onclick = () => {
      const open = info.classList.toggle("open");
      li.querySelector("span").textContent = open ? "-" : "+";
    };

    boatList.appendChild(li);
    boatList.appendChild(info);
    deleteSelect.innerHTML += `<option value="${b.id}">${b.name}</option>`;
  });
}

// Przyciski do modali
document.getElementById("addBoatBtn").onclick = () => (addModal.style.display = "flex");
document.getElementById("deleteBoatBtn").onclick = () => (deleteModal.style.display = "flex");

document.querySelectorAll(".close-modal").forEach(btn =>
  btn.onclick = () => {
    addModal.style.display = "none";
    deleteModal.style.display = "none";
  }
);

// Dodawanie łódki
document.getElementById("saveBoat").onclick = async () => {
  const name = document.getElementById("boatName").value;
  const model = document.getElementById("boatModel").value;
  const type = document.getElementById("boatType").value;
  const contact = document.getElementById("boatContact").value;

  try {
    const res = await fetch("/recorder/", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        name: name,
        created_at: Date.now(),
        model: model,
        boat_type: type,
        contact_number: contact
      })
    });
    const newBoat = await res.json();
    boats.push(newBoat);
    addModal.style.display = "none";
    renderBoats();
  } catch (err) {
    console.error("Błąd dodawania łódki:", err);
  }
  addModal.style.display = "none";
  renderBoats();
};


// Usuwanie łódki
document.getElementById("confirmDelete").onclick = async () => {
  const id = deleteSelect.value;
  try {
      await fetch(`/ui/fleet/${id}`, {
      method: "DELETE"
    });
    boats = boats.filter(b => b.id != id);
    deleteModal.style.display = "none";
    renderBoats();
  } catch (err) {
    console.error("Błąd usuwania łódki:", err);
  }
  deleteModal.style.display = "none";
  renderBoats();
};

loadBoats();