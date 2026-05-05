import { defineAstroFontProvider } from "astro/config";

export function LocalFonts() {
	return defineAstroFontProvider({
		entrypoint: "src/fonts/local_fonts.mjs"
	});
}
