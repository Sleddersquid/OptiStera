// Kanksje sette en if så at denne ikke langes når menyen ikke er ute
if (screen.width < 801) {
    console.log(screen.width);
      function showSidebar() {
          const sidebarEl = document.querySelector("ul.sidebar");
          sidebarEl.style.display = "flex";
      }
      function hideSidebar() {
          const sidebarEl = document.querySelector("ul.sidebar");
          sidebarEl.style.display = "none";
      }
  
      // CHATGPT
      document.querySelectorAll("h2.heading").forEach((heading) => {
          const span = heading.querySelector("span.line-title.left");
        
          if (span) {
            span.classList.replace("left", "right"); // Change class from "left" to "right"
            heading.appendChild(span); // Move <span> to the end of <h2>
          }
        });
  };