document.addEventListener("DOMContentLoaded", () => {
  const loginForm = document.querySelector("form");
  const emailInput = document.getElementById("email");
  const passwordInput = document.getElementById("password");

  loginForm.addEventListener("submit", async (e) => {
    e.preventDefault();

    const email = emailInput.value.trim();
    const password = passwordInput.value.trim();

    if(!email || !password){
      alert("Fill in both email and password.");
      return;
    }

    try {
      const res = await fetch("/login", {
        method: "POST",
        headers: {
          "Content-Type": "application/json"
        },
        body: JSON.stringify({ email, password }),
        credentials: "include" // umożliwia obsługę ciasteczek sesyjnych
      });

      if(res.status === 200){
        const data = await res.json();
        // zapisz dane w localStorage/sessionStorage jeśli chcesz
        sessionStorage.setItem("user", JSON.stringify(data.user));

        // przekieruj na stronę główną po zalogowaniu
        window.location.href = "/";
      } else if(res.status === 401){
        alert("Invalid email or password.");
      } else {
        alert("Login failed. Try again.");
      }
    } catch(err){
      console.error("Login error:", err);
      alert("Login error. Check console.");
    }
  });
});
