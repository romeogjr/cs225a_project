const template = document.createElement('template');
template.innerHTML = `
<div class="btn-group" role="group">
	<button type="button" class="btn btn-info">New online plot tab</button>
	<button type="button" class="btn btn-success">Open offline CSV plotter</button>
</div>
`;

class SaiInterfacesPlotButton extends HTMLElement {
	constructor() {
		super();
		this.template = template;
	}

	connectedCallback() {
		let template_node = this.template.content.cloneNode(true);

		// Add event listeners
		template_node.querySelector('.btn-info').addEventListener('click', this.newOnlinePlotTab);
		template_node.querySelector('.btn-success').addEventListener('click', this.openOfflineCSVPlotter);

		// append to document
		this.appendChild(template_node);
	}

	// Custom event listeners
	newOnlinePlotTab() {
		let options = {
			method: 'GET',
			headers: {
				'Content-Type': 'application/json'
			}
		};

		// Fetch the content of the link
		fetch("/html/online_plotter.html", options)
			.then(response => response.text())
			.then(htmlContent => {
				var newTab = window.open("", "_blank");
				if (newTab) {
					newTab.document.write(htmlContent);
					newTab.document.close();
					newTab.location.href = "html/online_plotter.html";
				} else {
					alert("Please allow pop-ups to open the link in a new tab.");
				}
			})
			.catch(error => {
				console.error("Error fetching the content:", error);
				alert("Failed to open the link. Please try again later.");
			});
	}

	openOfflineCSVPlotter() {
		const options = {
			method: 'POST',
			headers: {
				'Content-Type': 'application/json'
			}
		};

		fetch('/plot/offline', options)
			.then(response => {
				if (!response.ok) {
					throw new Error('Network response was not ok');
				}
				return response.text();
			})
			.catch(error => {
				console.error('There was a problem opening the offline csv plot program:', error);
			});
	}
}

// Define the custom element
customElements.define('sai-interfaces-plot-button', SaiInterfacesPlotButton);
