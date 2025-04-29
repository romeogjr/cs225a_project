import { EVENT_RESET_DISPLAYS, EVENT_RESET_ACTIVE_TABS_FROM_REDIS } from '../config.js';
import { post_redis_key_val, get_redis_val, wait_for_redis_val } from '../redis.js';

class SaiInterfacesTabs extends HTMLElement {
	constructor() {
		super();
		SaiInterfacesTabs.counter = (SaiInterfacesTabs.counter || 0) + 1;
		this.counter = SaiInterfacesTabs.counter;
	}

	setActiveTab(tabName) {
		const tabToActivate = this.querySelector(`.nav-link[id="${tabName}"]`);
		if (tabToActivate) {
			tabToActivate.click(); // Simulate a click event on the tab element
		} else {
			console.error(`Tab with id "${tabName}" not found.`);
		}
	}

	connectedCallback() {
		const name = this.getAttribute('name').replace(/\s+/g, '_');
		this.uniqueId = name + this.counter;

		const tabsContent = Array.from(this.children).filter(child => child.tagName === 'SAI-INTERFACES-TAB-CONTENT');
		const otherContent = Array.from(this.children).filter(child => child.tagName === 'SAI-INTERFACES-TAB-INLINE-CONTENT');

		const tabsOnTheLeft = this.hasAttribute('tabsOnTheLeft');
		const color = this.getAttribute('color') || 'rgb(0, 110, 255)';
		const key = this.getAttribute('key');

		// create map of tab values to names
		const tabValueToName = {};
		tabsContent.forEach((tabContent) => {
			const tabName = tabContent.getAttribute('name').replace(/\s+/g, '_');
			const tabValue = tabContent.getAttribute('value');
			tabValueToName[tabValue] = this.uniqueId + "_" + tabName + "-tab";
		});

		let activeTabName = sessionStorage.getItem(`${this.uniqueId}_activeTabName`);
		if (!activeTabName && tabsContent.length > 0) {
			// Set the first tab as active if no active tab is stored
			const tabName = tabsContent[0].getAttribute('name').replace(/\s+/g, '_');
			activeTabName = this.uniqueId + "_" + tabName + "-tab";
		}

		if (key) {
			get_redis_val(key).then(value => {
				if (value && tabValueToName[value]) {
					this.setActiveTab(tabValueToName[value]);
				}
			})
		}

		// Create the tabs HTML
		const tabsHTML = Array.from(tabsContent).map((tabContent, index) => {
			const tabNameDisplay = tabContent.getAttribute('name');
			const tabName = this.uniqueId + "_" + tabNameDisplay.replace(/\s+/g, '_') + "-tab";
			const isActive = tabName === activeTabName ? 'active' : '';
			const value = tabContent.getAttribute('value');
			return `<li class="nav-item">
                        <a class="nav-link ${isActive}" 
						id="${tabName}" data-bs-toggle="tab" 
						href="#${tabName}-content" role="tab" 
						aria-controls="${tabName}-content" 
						aria-selected="${isActive === 'active'}" 
						data-value="${value}">
							${tabNameDisplay}
						</a>
                    </li>`;
		}).join('');

		// Create the tab contents HTML
		const tabContentsHTML = Array.from(tabsContent).map((tabContent, index) => {
			const tabName = this.uniqueId + "_" + tabContent.getAttribute('name').replace(/\s+/g, '_') + "-tab";
			const isActive = tabName === activeTabName ? 'show active' : '';
			return `<div class="tab-pane fade ${isActive}" 
					id="${tabName}-content" role="tabpanel" 
					aria-labelledby="${tabName}">
                        ${tabContent.innerHTML}
                    </div>`;
		}).join('');

		// Construct the tabs structure
		const navType1 = tabsOnTheLeft ? 'nav-pills' : 'nav-tabs';
		const navType2 = tabsOnTheLeft ? 'flex-column' : 'flex-row';
		const tabsClass = tabsOnTheLeft ? 'col-md-2' : 'row';
		const contentClass = tabsOnTheLeft ? 'col-md-10' : 'row';

		const borderClass = tabsOnTheLeft ? 'border-right' : 'border-bottom';
		const paddingClass = tabsOnTheLeft ? 'padding-right' : 'padding-top';

		this.innerHTML = `
		<style>
			.${this.uniqueId} .nav-link.active {
				background-color: ${color};
				color: rgb(240, 240, 240);
			}

			.${this.uniqueId} .nav-link {
				background-color: rgb(240, 240, 240);
				color: ${color};
			}

			.${this.uniqueId} {
				${borderClass}: 2px solid ${color};
				${paddingClass}: 10px;
			}
		</style>

		<div class="row" name="tabs">
			<div class="${tabsClass}">
				<ul class="nav ${navType1} ${navType2} ${this.uniqueId}" id="${this.uniqueId}" role="tablist">
					${tabsHTML}
				</ul>
			</div>
			<div class="${contentClass}">
				<div class="tab-content" id="${this.uniqueId}-content">
					${tabContentsHTML}
				</div>
			</div>
		</div>
		`;

		// Add the other content to the tabs div
		let row = this.querySelector('.row');
		let tabsDiv = row.querySelector(`.${tabsClass}`);
		let ulDiv = tabsDiv.querySelector('ul');
		otherContent.forEach((content) => {
			ulDiv.appendChild(content);
		});

		// add event listener to refresh the active tab from redis if applicable
		document.addEventListener(EVENT_RESET_ACTIVE_TABS_FROM_REDIS, event => {
			if (key && event.detail.identifier != this.uniqueId) {
				get_redis_val(key).then(value => {
					if (value && tabValueToName[value]) {
						this.setActiveTab(tabValueToName[value]);
					}
				})
			}
		});

		// Add event listener to tabs for refreshing the page on tab switch 
		// and remembering which one is active
		this.querySelectorAll('.nav-link').forEach((tabLink) => {
			tabLink.addEventListener('show.bs.tab', async () => {
				// dispatch a tab reset event in case the active active tab key of subtabs has been changed
				document.dispatchEvent(new CustomEvent(EVENT_RESET_ACTIVE_TABS_FROM_REDIS, {
					bubbles: true,
					detail: {
						identifier: this.uniqueId,
					}
				}));

				if (tabLink.getAttribute('id').startsWith(this.uniqueId)) {
					// console.log('Tab clicked!', this.uniqueId, tabLink.getAttribute('id'));
					sessionStorage.setItem(`${this.uniqueId}_activeTabName`, tabLink.getAttribute('id'));
					
					if (key) {
						const value = tabLink.getAttribute('data-value');
						if (value && value !== 'null') {
							post_redis_key_val(key, value);
							await wait_for_redis_val(key, value);
						}
					}
					const resetDisplaysEvent = new CustomEvent(EVENT_RESET_DISPLAYS, {
						bubbles: true,
						detail: { message: 'Reset the displays' }
					});
					document.dispatchEvent(resetDisplaysEvent);


				}
			});
		});
	}
}

// for content to be displayed in the page and displayed when one tab is selected
class SaiInterfacesTabContent extends HTMLElement {
	constructor() {
		super();
	}
}

// for content to appear in the tab area iteslf, at al times
class SaiInterfacesTabInlineContent extends HTMLElement {
	constructor() {
		super();
	}
}

customElements.define('sai-interfaces-tabs', SaiInterfacesTabs);
customElements.define('sai-interfaces-tab-content', SaiInterfacesTabContent);
customElements.define('sai-interfaces-tab-inline-content', SaiInterfacesTabInlineContent);
