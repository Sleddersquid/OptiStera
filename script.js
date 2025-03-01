// Kanksje sette en if så at denne ikke langes når menyen ikke er ute
console.log(screen.width);

if (screen.width < 900) {
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

      // CHATGPT
      document.addEventListener("DOMContentLoaded", function () {
        const timelineEvents = document.querySelectorAll(".timeline-event");
      
        const observerOptions = {
          root: null, // Uses viewport
          rootMargin: "-60% 0px -40% 0px", 
          threshold: 0, // 40% of the element must be visible
        };
      
        const observer = new IntersectionObserver((entries) => {
          entries.forEach((entry) => {
              console.log(entry.target.classList);
            if (entry.isIntersecting) {
              entry.target.classList.add("visible");
              // entry.target.style = "background-color: #000;"; // DEbug
            } else {
              entry.target.classList.remove("visible"); // Optional: remove when out of view
              // entry.target.style = "background-color: #fff;"; // Debug
            }
          });
        }, observerOptions);
      
        timelineEvents.forEach((el) => observer.observe(el));
      });
};