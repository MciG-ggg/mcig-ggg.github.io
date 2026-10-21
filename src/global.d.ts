import type mediumZoom from "medium-zoom";

declare global {
	interface Window {
		/** medium-zoom instance. Created once so it does not stack document listeners. */
		__mediumZoom?: ReturnType<typeof mediumZoom>;
		/** Listeners registered once for the whole session, replaced on navigation. */
		__tocRegistry?: { run: () => void; updateProgress: () => void; bound: boolean };
		__positionRegistry?: { updateProgress: () => void; bound: boolean };
		__articleActionsRegistry?: { bound: boolean };
		__baseRegistry?: { init: () => void; bound: boolean };
	}
}
