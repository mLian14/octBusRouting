package processor;

/**
 * @auther lianmeng
 * @create 23.06.23
 */
import shapes.PseudoBase;

import java.util.*;

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

    public static List<ArrayList<PseudoBase>> generateRandomPermutations(ArrayList<PseudoBase> input, int desiredCount) {
        List<ArrayList<PseudoBase>> permutations = new ArrayList<>();
        int totalPermutations = Math.min(factorial(input.size()), desiredCount);

        if (totalPermutations == factorial(input.size())) {
            // Generate all possible permutations in case desired count equals total count
            return generatePermutations(input);
        }

        Set<List<PseudoBase>> uniquePermutations = new HashSet<>();
        Random random = new Random();

        while (uniquePermutations.size() < totalPermutations) {
            ArrayList<PseudoBase> randomPermutation = new ArrayList<>(input);
            for (int i = 0; i < input.size() - 1; i++) {
                int randomIndex = random.nextInt(input.size() - i) + i;
                swap(randomPermutation, i, randomIndex);
            }
            uniquePermutations.add(randomPermutation);
        }

        for (List<PseudoBase> uniquePermutation : uniquePermutations) {
            permutations.add(new ArrayList<>(uniquePermutation));
        }

        return permutations;
    }

    private static int factorial(int n) {
        if (n <= 1)
            return 1;
        return n * factorial(n - 1);
    }
}

