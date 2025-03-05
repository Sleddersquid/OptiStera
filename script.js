function onResize() {
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
  
        // CHATGPT
        document.addEventListener("DOMContentLoaded", function () {
          const timelineEvents = document.querySelectorAll(".timeline-event");
        
          const observerOptions = {
            root: null, // Uses viewport
            rootMargin: "-50% 0px -50% 0px", 
            threshold: 0,
          };
        
          const observer = new IntersectionObserver((entries) => {
            entries.forEach((entry) => {
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
}

window.addEventListener('resize', onResize);

onResize(); // For first page load

