<script lang="ts">
import { onMount } from "svelte";
import Icon from "$components/Icon.svelte";

let { dark = $bindable(false) } = $props();

/**
 * Apply theme to the DOM. Does not persist: persistence records an explicit
 * user choice, and the system preference must not count as one.
 * @param on whether to enable dark mode
 */
function turnDark(on: boolean) {
	let theme = (dark = on) ? "dark" : "light";
	// Update CSS custom properties via data attribute
	document.documentElement.dataset.theme = theme;
	// Let theme-aware widgets (mermaid, charts) redraw themselves
	document.dispatchEvent(new CustomEvent("theme:change", { detail: theme }));
}

/**
 * Handle theme toggle with animated transition effect
 * @param event Mouse event for click coordinates
 */
function triggerDark(event: MouseEvent) {
	// A click is an explicit choice, so persist it.
	const trigger = () => {
		turnDark(!dark);
		localStorage.setItem("theme", dark ? "dark" : "light");
	};

	let transition: ViewTransition;
	if (!(transition = document.startViewTransition?.(trigger))) return trigger(); // Compatibility check

	// Get click coordinates for radial animation origin
	const x = event.clientX;
	const y = event.clientY;
	transition.ready.then(() => {
		// Create expanding circle animation from click point
		const path = [`circle(0% at ${x}px ${y}px)`, `circle(130% at ${x}px ${y}px)`];
		document.documentElement.animate(
			{
				// Reverse animation direction based on theme transition
				clipPath: dark ? [...path].reverse() : path
			},
			{
				duration: 400,
				easing: "ease-in",
				// Keep end state after animation completes to avoid flicker
				fill: "forwards",
				// Target different pseudo-elements for incoming/outgoing content
				pseudoElement: dark ? "::view-transition-old(root)" : "::view-transition-new(root)"
			}
		);
	});
}

onMount(() => {
	// Detect system color scheme preference
	const mode = window.matchMedia("(prefers-color-scheme: dark)");
	const theme = localStorage.getItem("theme");

	// Use stored preference or fallback to system preference
	turnDark(theme ? theme === "dark" : mode.matches);
	// Follow the system only while the user has made no explicit choice.
	// An explicit choice in localStorage wins over later system changes.
	mode.addEventListener("change", ({ matches }) => {
		if (!localStorage.getItem("theme")) turnDark(matches);
	});
});
</script>

<button class="items-center" aria-label="Toggle dark theme" onclick={triggerDark}>
	{#if dark}
		<Icon name="lucide--moon" />
	{:else}
		<Icon name="lucide--sun" />
	{/if}
</button>
