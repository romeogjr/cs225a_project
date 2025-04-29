/**
 * Helper functions to get/set Redis keys.
 * @module ./redis
 */

/**
 * Sets a Redis key to the specified value.
 * @param {string} key The Redis key to set
 * @param {*} val The value with which to set the Redis key
 * @returns {Promise} A promise that returns no data on success.
 */
export function post_redis_key_val(key, val) {
	let fetchOptions = {
		method: 'POST',
		headers: new Headers({ 'Content-Type': 'application/json' }),
		mode: 'same-origin',
		body: JSON.stringify({ key, val })
	};

	return fetch('/redis', fetchOptions)
		.catch(error => console.error('error posting redis key: ' + key + '. Error: ' + error));
}

/**
 * Gets one or more Redis keys.
 * @param {(string|string[])} key Redis key(s) to query
 * @returns {Promise<Object>} A promise containing Redis key: value pairs.
 */
export function get_redis_val(key) {
	let fetchOptions = {
		method: 'GET',
		headers: new Headers({ 'Content-Type': 'application/json' }),
		mode: 'same-origin'
	};

	let params = new URLSearchParams({ key: JSON.stringify(key) });

	return fetch('/redis?' + params.toString(), fetchOptions)
		.then(response => response.json())
		.catch(error => console.error('error posting redis key: ' + key + '. Error: ' + error));
}

/**
 * Waits for a Redis key to have a specific value.
 * @param {string} key The Redis key to check
 * @param {*} val The value to wait for
 * @returns {Promise} A promise that resolves when the Redis key has the specified value
 */
export function wait_for_redis_val(key, val) {
	return new Promise(resolve => {
		const checkValue = () => {
			get_redis_val(key).then(value => {
				// console.log(value);
				if (value != val) {
					setTimeout(checkValue, 50);
				} else {
					resolve();
				}
			});
		};
		checkValue();
	});
};

/**
 * Gets all Redis keys.
 * @returns {Promise<string[]>} A promise that on resolution has a string[] 
 * of all available Redis keys to query and set.
 */
export function get_redis_all_keys() {
	let fetchOptions = {
		method: 'GET',
		headers: new Headers({ 'Content-Type': 'application/json' }),
		mode: 'same-origin'
	};

	return fetch('/redis/keys', fetchOptions)
		.then(response => response.json())
		.catch(error => console.error('error posting redis key: ' + key + '. Error: ' + error));
}