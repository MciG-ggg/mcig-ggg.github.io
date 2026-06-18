<script lang="ts">
import { getRelativeLocaleUrl } from "astro:i18n";
import { monolocale } from "$config";
import Time from "$utils/time";
import i18nit from "$i18n";

let {
	locale,
	notes,
	jottings,
	weeks = 20,
	showEntries = true
}: { locale: string; notes: any[]; jottings: any[]; weeks?: number; showEntries?: boolean } = $props();

const days = weeks * 7; // Convert weeks to days for heatmap
const monthFormatter = new Intl.DateTimeFormat(locale, { month: "short" });

// Initialize translation function for current locale
const t = i18nit(locale);

// Get this week's Saturday as reference point for calculating relative dates
const now = new Date();
// Get day of week in the configured timezone (0 = Sunday, 6 = Saturday)
const start = Time.addDays(now, (6 - Time.weekday(now)) % 7);

// Create 140-day heatmap data structure (roughly 4+ months of activity)
// Each day contains: date, empty arrays for notes and jottings
const heatmap = Array.from({ length: days }, (_, day) => ({
	date: Time.subtractDays(start, day), // Calculate date going backwards from today
	notes: [] as any[], // Notes published on this day
	jottings: [] as any[] // Jottings published on this day
}));

// Populate heatmap with notes data
notes.forEach(note => {
	// Calculate how many days ago this note was published
	let gap = Time.diffDays(start, note.data.timestamp);

	// Only include notes from the last 100 days
	if (0 <= gap && gap < days) heatmap[gap].notes.push(note);
});

// Populate heatmap with jottings data
jottings.forEach(jotting => {
	// Calculate how many days ago this jotting was published
	let gap = Time.diffDays(start, jotting.data.timestamp);

	// Only include jottings from the last 100 days
	if (0 <= gap && gap < days) heatmap[gap].jottings.push(jotting);
});

const orderedHeatmap = [...heatmap].reverse();
const weeksData = Array.from({ length: weeks }, (_, week) => orderedHeatmap.slice(week * 7, week * 7 + 7));
let lastMonth = "";
let lastLabelIndex = -4;
const monthLabels = weeksData.map((week, index) => {
	const month = monthFormatter.format(week[0].date);
	if (month !== lastMonth && index - lastLabelIndex >= 3) {
		lastMonth = month;
		lastLabelIndex = index;
		return month;
	}
	lastMonth = month;
	return "";
});
const total = orderedHeatmap.reduce((sum, day) => sum + day.notes.length + day.jottings.length, 0);
const firstYear = orderedHeatmap[0].date.getFullYear();
const lastYear = orderedHeatmap[orderedHeatmap.length - 1].date.getFullYear();
const yearRange = firstYear === lastYear ? `${firstYear}` : `${firstYear} ~ ${lastYear}`;
const totalLabel = locale.startsWith("zh") ? `${total} 则记录，${yearRange}` : `${total} records in ${yearRange}`;
const lessLabel = locale.startsWith("zh") ? "少" : "Less";
const moreLabel = locale.startsWith("zh") ? "多" : "More";

function level(count: number) {
	if (count >= 4) return 4;
	if (count >= 3) return 3;
	if (count >= 2) return 2;
	if (count >= 1) return 1;
	return 0;
}
</script>

<section class="heatmap" style={`--week-count: ${weeks}`}>
	<div class="heatmap-months" aria-hidden="true">
		{#each monthLabels as label}
			<span>{label}</span>
		{/each}
	</div>

	<div class="heatmap-grid" aria-label={totalLabel}>
		{#each weeksData as week}
			<div class="heatmap-week">
				{#each week as day}
					{@const number = day.notes.length + day.jottings.length}
					<figure class="heatmap-day group/pop">
						<i class="heatmap-cell" data-level={level(number)} aria-label={`${number} writing records on ${Time.date.locale(day.date, locale)}`}></i>

						<div class="heatmap-tooltip pop">
							<time>{Time.date.locale(day.date, locale)}</time>
							{#if number > 0}
								{#if showEntries}
									{#if day.notes.length > 0}
										<p>{t("home.heatmap.note", { count: day.notes.length })}</p>
										<ul>
											{#each day.notes as note}
												<a href={getRelativeLocaleUrl(locale, `/note/${monolocale ? note.id : note.id.split("/").slice(1).join("/")}`)} aria-label={note.data.title} class="link">{note.data.title}</a>
											{/each}
										</ul>
									{/if}
									{#if day.jottings.length > 0}
										<p>{t("home.heatmap.jotting", { count: day.jottings.length })}</p>
										<ul>
											{#each day.jottings as jotting}
												<a href={getRelativeLocaleUrl(locale, `/jotting/${monolocale ? jotting.id : jotting.id.split("/").slice(1).join("/")}`)} aria-label={jotting.data.title} class="link">{jotting.data.title}</a>
											{/each}
										</ul>
									{/if}
								{:else}
									{#if day.notes.length > 0}
										<p>{t("home.heatmap.note", { count: day.notes.length })}</p>
									{/if}
									{#if day.jottings.length > 0}
										<p>{t("home.heatmap.jotting", { count: day.jottings.length })}</p>
									{/if}
								{/if}
							{:else}
								<p>{t("home.heatmap.empty")}</p>
							{/if}
						</div>
					</figure>
				{/each}
			</div>
		{/each}
	</div>

	<footer class="heatmap-footer">
		<p>{totalLabel}</p>
		<div class="heatmap-legend" aria-hidden="true">
			<span>{lessLabel}</span>
			{#each [0, 1, 2, 3, 4] as item}
				<i class="heatmap-cell" data-level={item}></i>
			{/each}
			<span>{moreLabel}</span>
		</div>
	</footer>
</section>

<style>
	.heatmap {
		--heatmap-cell: 0.75rem;
		--heatmap-gap: 0.25rem;
		--heatmap-empty: #ebedf0;
		--heatmap-1: #9be9a8;
		--heatmap-2: #40c463;
		--heatmap-3: #30a14e;
		--heatmap-4: #216e39;
		--heatmap-border: #d8d8d2;
		--heatmap-tooltip-bg: #1d1d1b;
		--heatmap-tooltip-text: #fffffd;
		display: grid;
		gap: 0.55rem;
		min-width: calc(var(--week-count) * var(--heatmap-cell) + (var(--week-count) - 1) * var(--heatmap-gap));
		color: var(--secondary-color);
		font-family: var(--font-mono);
		font-size: 0.75rem;
	}

	:global([data-theme="dark"]) .heatmap {
		--heatmap-empty: #262b30;
		--heatmap-1: #0e4429;
		--heatmap-2: #006d32;
		--heatmap-3: #26a641;
		--heatmap-4: #39d353;
		--heatmap-border: #30363d;
		--heatmap-tooltip-bg: #f0f0ec;
		--heatmap-tooltip-text: #10100e;
		color: #c9c9c3;
	}

	.heatmap-months,
	.heatmap-grid {
		display: grid;
		grid-template-columns: repeat(var(--week-count), var(--heatmap-cell));
		gap: var(--heatmap-gap);
	}

	.heatmap-months span {
		width: max-content;
		min-height: 1rem;
		color: var(--remark-color);
		line-height: 1;
	}

	:global([data-theme="dark"]) .heatmap-months span {
		color: #c0c0b8;
	}

	.heatmap-week {
		display: grid;
		grid-template-rows: repeat(7, var(--heatmap-cell));
		gap: var(--heatmap-gap);
	}

	.heatmap-day {
		position: relative;
		display: block;
		width: var(--heatmap-cell);
		height: var(--heatmap-cell);
	}

	.heatmap-cell {
		display: block;
		width: var(--heatmap-cell);
		height: var(--heatmap-cell);
		border: 1px solid var(--heatmap-border);
		border-radius: 0.1875rem;
		background: var(--heatmap-empty);
	}

	.heatmap-cell[data-level="1"] {
		background: var(--heatmap-1);
	}

	.heatmap-cell[data-level="2"] {
		background: var(--heatmap-2);
	}

	.heatmap-cell[data-level="3"] {
		background: var(--heatmap-3);
	}

	.heatmap-cell[data-level="4"] {
		background: var(--heatmap-4);
	}

	.heatmap-tooltip {
		position: absolute;
		inset-inline-start: 50%;
		bottom: calc(100% + 0.45rem);
		display: flex;
		width: max-content;
		max-width: 16rem;
		flex-direction: column;
		gap: 0.25rem;
		border: 1px solid var(--heatmap-border);
		border-radius: 0.35rem;
		padding: 0.45rem 0.6rem;
		color: var(--heatmap-tooltip-text);
		background: var(--heatmap-tooltip-bg);
		box-shadow: 0 0.4rem 1rem color-mix(in srgb, var(--primary-color) 16%, transparent);
		line-height: 1.35;
		transform: translateX(-50%);
		pointer-events: none;
	}

	.heatmap-tooltip time {
		font-weight: 700;
	}

	.heatmap-tooltip ul {
		display: grid;
		gap: 0.15rem;
	}

	.heatmap-footer {
		display: flex;
		align-items: center;
		justify-content: space-between;
		gap: 1rem;
		color: var(--secondary-color);
	}

	:global([data-theme="dark"]) .heatmap-footer {
		color: #d4d4ce;
	}

	.heatmap-legend {
		display: flex;
		align-items: center;
		gap: 0.25rem;
	}

	.heatmap-legend .heatmap-cell {
		width: 0.68rem;
		height: 0.68rem;
	}
</style>
