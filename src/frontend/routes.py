from flask import render_template, request, redirect, url_for, flash
import requests

def init_routes(app):

    @app.route("/")
    def home():
        return render_template("index.html")

    @app.route("/login", methods=["GET"])
    def login_page():
        return render_template("login.html")

    @app.route("/login", methods=["POST"])
    def login_submit():
        email = request.form.get("email")
        password = request.form.get("password")

        print(f"Login próba: {email}")
        return redirect(url_for("home"))

    @app.route("/register", methods=["GET", "POST"])
    def register_page():
        if request.method == "GET":
            return render_template("register.html")

        username = request.form.get("username")
        email = request.form.get("email")
        password = request.form.get("password")
        confirm = request.form.get("confirm")

        if password != confirm:
            flash("Passwords don't match!")
            return render_template("register.html")

        backend_url = "http://localhost:8080/api/user/register"

        payload = {
            "username": username,
            "email": email,
            "password": password,
            "phone": None
        }

        try:
            response = requests.post(backend_url, json=payload)
        except Exception as e:
            print("Backend connection error:", e)
            flash("Cannot connect to backend. Is FastAPI running?")
            return render_template("register.html")

        if response.status_code == 200:
            flash("Registration successful!")
            return redirect(url_for("login_page"))

        detail = response.json().get("detail", "Unknown error")
        flash(f"Registration failed: {detail}")
        return render_template("register.html")

    @app.route("/my_fleet")
    def my_fleet():
        return render_template("my_fleet.html")

    @app.route("/incidents")
    def incidents():
        return render_template("incidents.html")
