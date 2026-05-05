import { defineFontProvider } from "unifont";

const LOCAL_FONTS = {
	"Noto Serif": [400, 700],
	"Noto Serif SC": [400, 700],
	"Noto Serif JP": [400, 700],
	"Playwrite MX": [100],
	"Maple Mono NF CN": [400],
	"The Peak Font Plus": [400]
};

export const provider = defineFontProvider("Local Fonts", async () => ({
	resolveFont: async fontFamily => {
		const weights = LOCAL_FONTS[fontFamily];
		if (!weights) return { fonts: [] };

		return {
			fonts: weights.map(weight => ({
				src: [{ name: fontFamily }],
				weight
			}))
		};
	},
	listFonts: () => Object.keys(LOCAL_FONTS)
}));
