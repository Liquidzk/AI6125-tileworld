package tileworld;

import java.util.concurrent.ThreadLocalRandom;
import tileworld.environment.TWEnvironment;

public class TileworldMain {

    public static void main(String args[]) {
        int overallScore = 0;
        int iteration = Integer.getInteger("tileworld.iterations", 10);

        for (int i = 0; i < iteration; i++) {
            int seed = useConfiguredSeed() ? Parameters.seed + i : ThreadLocalRandom.current().nextInt(1, Integer.MAX_VALUE);
            System.out.println("Seed: " + seed);

            TWEnvironment tw = new TWEnvironment(seed);
            tw.start();

            long steps = 0;
            while (steps < Parameters.endTime) {
                if (!tw.schedule.step(tw)) {
                    break;
                }
                steps = tw.schedule.getSteps();
            }

            System.out.println("The final reward is: " + tw.getReward());
            overallScore += tw.getReward();
            tw.finish();
        }

        System.out.println("The average reward is: " + ((float) overallScore / iteration));
        System.exit(0);
    }

    private static boolean useConfiguredSeed() {
        return Boolean.getBoolean("tileworld.useConfiguredSeed");
    }
}
