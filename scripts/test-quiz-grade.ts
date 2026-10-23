/**
 * Check the grading rules of the quiz module.
 *
 * The rules are pure functions, so they run without a browser. The check
 * covers the cases that decide whether a reader gets the points.
 *
 * Usage: pnpm exec tsx scripts/test-quiz-grade.ts
 */
import { gradeChoice, gradeFill } from "../src/utils/interactive/grade";

let failed = 0;

const check = (name: string, got: boolean, want: boolean): void => {
	if (got === want) {
		console.log(`  ok  ${name}`);
		return;
	}
	failed += 1;
	console.log(`  FAIL ${name}: got ${got}, want ${want}`);
};

console.log("choice");
check("单选正确", gradeChoice(["2"], "2"), true);
check("单选错误", gradeChoice(["1"], "2"), false);
check("没选", gradeChoice([], "2"), false);
check("选了多个", gradeChoice(["1", "2"], "2"), false);
check("多选全对", gradeChoice(["1", "3"], "1,3"), true);
check("多选漏一个", gradeChoice(["1"], "1,3"), false);
check("多选多一个", gradeChoice(["1", "2", "3"], "1,3"), false);
check("多选顺序无关", gradeChoice(["3", "1"], "1,3"), true);
check("key 里有空格", gradeChoice(["2"], " 2 "), true);

console.log("fill");
check("全对", gradeFill(["非极大值抑制", "双阈值滞后"], "非极大值抑制|双阈值滞后"), true);
check("错一个", gradeFill(["非极大值抑制", "Sobel"], "非极大值抑制|双阈值滞后"), false);
check("少一个空", gradeFill(["非极大值抑制"], "非极大值抑制|双阈值滞后"), false);
check("首尾空格不算错", gradeFill([" 非极大值抑制 ", "双阈值滞后"], "非极大值抑制|双阈值滞后"), true);
check("空里有逗号", gradeFill(["a, b"], "a, b"), true);
check("没填", gradeFill(["", "双阈值滞后"], "非极大值抑制|双阈值滞后"), false);

if (failed > 0) {
	console.log(`\n${failed} 个检查没过`);
	process.exit(1);
}
console.log("\n全部通过");
