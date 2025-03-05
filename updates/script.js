function onResize() {
    if (screen.width < 801) {
        function showSidebar() {
            const sidebarEl = document.querySelector("ul.sidebar");
            sidebarEl.style.display = "flex";
        }
        function hideSidebar() {
            const sidebarEl = document.querySelector("ul.sidebar");
            sidebarEl.style.display = "none";
        }
    };
}

window.addEventListener('resize', onResize);

onResize(); // For first page load