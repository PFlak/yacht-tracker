// Lista łodzi – bez statusu
const boats = [
  { name: "Alicja", price: 120, img: "https://source.unsplash.com/400x300/?sailboat" },
  { name: "Igła", price: 90, img: "https://source.unsplash.com/400x301/?yacht" },
  { name: "Rybak", price: 70, img: "https://source.unsplash.com/400x302/?boat" },
  { name: "Irmina", price: 80, img: "https://source.unsplash.com/400x303/?sea" },
  { name: "Posejdon", price: 140, img: "https://source.unsplash.com/400x304/?marina" },
];

// Elementy DOM — NAZWA ZMIENIONA NA boat-list
const boatList = document.getElementById("boat-list");
const addModal = document.getElementById("addBoatModal");
const deleteModal = document.getElementById("deleteBoatModal");
const deleteSelect = document.getElementById("deleteBoatSelect");

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
      Price: ${b.price} PLN/h<br>
      <img src="${b.img}" width="200" style="margin-top:8px;border-radius:8px">
    `;

    li.onclick = () => {
      const open = info.classList.toggle("open");
      li.querySelector("span").textContent = open ? "-" : "+";
    };

    boatList.appendChild(li);
    boatList.appendChild(info);
    deleteSelect.innerHTML += `<option value="${b.name}">${b.name}</option>`;
  });
}
renderBoats();

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
document.getElementById("saveBoat").onclick = () => {
  boats.push({
    name: document.getElementById("boatName").value,
    price: document.getElementById("boatPrice").value,
    img: `https://source.unsplash.com/400x30${Math.floor(Math.random() * 9)}/?sail`
  });

  addModal.style.display = "none";
  renderBoats();
};

// Usuwanie łódki
document.getElementById("confirmDelete").onclick = () => {
  const name = deleteSelect.value;
  const idx = boats.findIndex(b => b.name === name);
  if (idx !== -1) boats.splice(idx, 1);
  deleteModal.style.display = "none";
  renderBoats();
};
