/**
 * Escape text for use in HTML.
 *
 * The same rules apply to text nodes and to attribute values, so one function
 * covers both. The apostrophe is escaped too, because the values end up
 * inside single-quoted inline styles in some modules.
 *
 * The replacements use numeric character references so the file stays free of
 * named HTML entities.
 */
export const escapeHtml = (value: string): string =>
	value.replace(/&/g, "&#38;").replace(/</g, "&#60;").replace(/>/g, "&#62;").replace(/"/g, "&#34;").replace(/'/g, "&#39;");
