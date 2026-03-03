from flask import render_template, request, redirect, url_for

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

    @app.route("/register", methods=["GET"])
    def register_page():
        return render_template("register.html")

    @app.route("/my_fleet")
    def my_fleet():
        return render_template("my_fleet.html")

    @app.route("/incidents")
    def incidents():
        return render_template("incidents.html")
