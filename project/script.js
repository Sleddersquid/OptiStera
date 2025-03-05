function showSidebar() {
  const sidebarEl = document.querySelector("ul.sidebar");
  sidebarEl.style.display = "flex";
}
function hideSidebar() {
  const sidebarEl = document.querySelector("ul.sidebar");
  sidebarEl.style.display = "none";
}

function onResize() {
  if (screen.width < 801) {
    console.log(screen.width);

    // CHATGPT
    document.querySelectorAll("h2.heading").forEach((heading) => {
      const span = heading.querySelector("span.line-title.left");
      
      if (span) {
        span.classList.replace("left", "right"); // Change class from "left" to "right"
        heading.appendChild(span); // Move <span> to the end of <h2>
      }
    });
  };
}

window.addEventListener('resize', onResize);

onResize(); // For first page load