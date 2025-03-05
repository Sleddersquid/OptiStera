if (screen.width < 801 || true) {
    console.log(screen.width);
    function showSidebar() {
        const sidebarEl = document.querySelector("ul.sidebar");
        sidebarEl.style.display = "flex";
    }
    function hideSidebar() {
        const sidebarEl = document.querySelector("ul.sidebar");
        sidebarEl.style.display = "none";
    }
};