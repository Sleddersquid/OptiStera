// Kanksje sette en if så at denne ikke langes når menyen ikke er ute
if (screen.width < 801 || true) {
    function showSidebar() {
        const sidebarEl = document.querySelector("ul.sidebar");
        sidebarEl.style.display = "flex";
    }
    function hideSidebar() {
        const sidebarEl = document.querySelector("ul.sidebar");
        sidebarEl.style.display = "none";
    }
};