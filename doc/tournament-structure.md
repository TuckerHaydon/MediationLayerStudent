# Tournament Structure

## Important Dates
- May 1st, Pre-tournament Start Date. First day that students may submit
  binaries
- May 2nd, 1st pre-tournament Evaluation Date. Binaries and tournament standing
  are evaluated at 9 am. Binaries will be updated and evaluated every day
  at 9 am thereafter until the end of the pre-tournament.
- May 8th, Test image dataset released.
- May 15th, Final day of pre-tournament. Final evaluation at 9 am. Final map of
  arena released (this one will be used on the day of the tournament).
- May 17th, Tournament day. 9 am

## Tournament Binaries
To allow teams to evaluate their standing with respect to the other teams, but
prevent code-sharing, the pre-tournament will employ binaries. Each GameEngine
repository is set up to build a binary called "student_autonomy_protocol" and
place it into the GameEngine/bin folder. Teams will be required to submit this
binary for tournament evaluation.

### Binary Submission
Members from each team will be added to a private gitlab repository owned by
Tucker. To submit a binary, teams must copy their "student_autonomy_protocol"
binary into the git repository and push it to gitlab.com. Tucker will pull these
binaries every day at 9 am and evaluate tournament standings.

### Public Binaries
All submitted binaries become are public. All teams may view their competition's
submissions by cloning and updating the following git repository:
```bash
git clone https://gitlab.com/tuckerhaydon/tournamentbinaries.git TournamentBinaries
```

To evaluate an opponent's strategy, launch all tournament executables as normal,
but substitute an opponent's "student_autonomy_protocol".

### Binary Evaluation
Tucker will evaluate the submitted binaries every day around 9 am and send out
a canvas announcement with the current tournament standings. The submitted
binaries will be uploaded to the public TournamentBinaries folder so that
students may view the submitted strategies themselves.

## Maps
Three maps are provided for autonomy protocol evaluation. They are located under
GameEngine/resources/maps/. To change the map, the initial location of
quadcopter, or the locations of the balloons, change the corresponding
parameters in the GameEngine/run/params.yaml file and reload them.

The pre-tournament will be evaluated on the pretournament.map file. The other
maps in the directory are provided for convenience and testing.

A final map will be released at th end of the pre-tournament. This map will be
used for the final, real-life tournament evaluation.
