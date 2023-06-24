package processor;

/**
 * @auther lianmeng
 * @create 23.06.23
 */
import shapes.PseudoBase;

import java.util.ArrayList;
import java.util.List;

public class PermutationGenerator {

    public static List<ArrayList<PseudoBase>> generatePermutations(ArrayList<PseudoBase> input) {
        List<ArrayList<PseudoBase>> permutations = new ArrayList<>();
        generatePermutations(input, 0, permutations);
        return permutations;
    }

    private static void generatePermutations(ArrayList<PseudoBase> input, int start, List<ArrayList<PseudoBase>> result) {
        if (start >= input.size()) {
            // Found a valid permutation
            result.add(new ArrayList<>(input));
            return;
        }

        for (int i = start; i < input.size(); i++) {
            // Swap elements at indices start and i
            swap(input, start, i);

            // Recursively generate permutations
            generatePermutations(input, start + 1, result);

            // Restore the original order by swapping back
            swap(input, start, i);
        }
    }

    private static void swap(ArrayList<PseudoBase> input, int i, int j) {
        PseudoBase temp = input.get(i);
        input.set(i, input.get(j));
        input.set(j, temp);
    }
}

