document.addEventListener("DOMContentLoaded", () => {
  const form = document.querySelector("form");
  const usernameInput = document.getElementById("username");
  const emailInput = document.getElementById("email");
  const passwordInput = document.getElementById("password");
  const confirmInput = document.getElementById("confirm");

  form.addEventListener("submit", async (e) => {
    e.preventDefault();

    const username = usernameInput.value.trim();
    const email = emailInput.value.trim();
    const password = passwordInput.value;
    const confirm = confirmInput.value;

    if (!username || !email || !password || !confirm) {
      alert("Please fill all fields.");
      return;
    }

    if (password !== confirm) {
      alert("Passwords do not match!");
      return;
    }

    try {
      const res = await fetch("/register", {
        method: "POST",
        headers: {
          "Content-Type": "application/json"
        },
        body: JSON.stringify({ username, email, password }),
        credentials: "include" // jeśli backend ustawia ciasteczko sesyjne
      });

      if (res.status === 200 || res.status === 201) {
        const data = await res.json();
        // Możesz od razu zapisać dane użytkownika w sessionStorage
        sessionStorage.setItem("user", JSON.stringify(data.user));

        alert("Registration successful! Redirecting to home...");
        window.location.href = "/";
      } else if (res.status === 400 || res.status === 409) {
        const err = await res.json();
        alert(err.message || "Registration failed.");
      } else {
        alert("Unexpected error. Try again later.");
      }
    } catch (err) {
      console.error("Registration error:", err);
      alert("Registration failed. Check console.");
    }
  });
});
