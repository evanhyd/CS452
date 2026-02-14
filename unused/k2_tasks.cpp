#include "k2_tasks.h"
#include "debug.h"
#include "fmt.h"
#include "kit_algorithm.h"
#include "name_server.h"
#include "ring_buffer.h"
#include "syscalls.h"
#include "task_queue.h"
#include "timer.h"
#include "uart.h"
#include <cstdint>
#include <iterator>

namespace {

// Game config.
constexpr const char* RPS_SERVER_NAME = "rps_server";
constexpr size_t MAX_QUEUE_SIZE = 2;
constexpr size_t MAX_LOBBY = 32;

// Magic enums.
enum class RPSMessageType : uint8_t {
  SIGNUP,
  LOBBY_ASSIGNED,
  NO_AVAILABLE_LOBBY,
  PLAY,
  MATCH_RESULT,
  QUIT,
};
enum class PlayType : uint8_t { EMPTY, ROCK, PAPER, SCISSORS };

const char* playToString(PlayType play) {
  static constexpr const char* playNames[] = {"empty", "rock", "paper", "scissors"};
  auto idx = static_cast<uint8_t>(play);
  if (idx >= std::size(playNames)) {
    logError("invalid PlayType");
  }
  return playNames[idx];
}

// Request to sign up to the RPS game.
struct SignUpMessage {};

// Found a match response.
struct LobbyAssignedMessage {
  int lobbyId;
};

// No available lobby.
struct NoAvailableLobbyMessage {};

// Request to play the RPS move in a given lobby.
struct PlayMessage {
  int lobbyId;
  PlayType play;
};

// Match result response.
struct MatchResultMessage {
  PlayType myPlay;
  PlayType opponentPlay;
};

// Request to quit the queue or lobby.
struct QuitMessage {
  int lobbyId;
};

// Composite messages.
struct RPSMessage {
  RPSMessageType type;
  union {
    SignUpMessage signUpMessage;
    LobbyAssignedMessage lobbyAssignedMessage;
    NoAvailableLobbyMessage noAvailableLobbyMessage;
    PlayMessage playMessage;
    MatchResultMessage matchResultMessage;
    QuitMessage quitMessage;
  };
};

struct Lobby {
  bool isPlaying;
  int player1Tid;
  int player2Tid;
  PlayType player1Play;
  PlayType player2Play;
};

void rpsServerTask() {
  // Register to the name server.
  if (RegisterAs(RPS_SERVER_NAME) < 0) {
    Uart::syncPrint("[SERVER] failed to register to name server\r\n");
    return;
  }

  const auto doReply = [](int tid, const RPSMessage& msg) {
    if (int ret = ::Reply(tid, reinterpret_cast<const char*>(&msg), sizeof(RPSMessage)); ret < 0) {
      static char buffer[128];
      kit::formatString(buffer, "[SERVER] failed to reply to client %d (code %d)\r\n", tid, ret);
      Uart::syncPrint(buffer);
    }
  };

  RingBuffer<int, MAX_QUEUE_SIZE> playerQueue;
  Lobby lobbies[MAX_LOBBY] = {};

  // Handlers
  const auto signUpHandler = [&](int senderTid) {
    playerQueue.push(senderTid);
    if (playerQueue.size() < 2) {
      return;
    }
    int player1Tid = playerQueue.pop();
    int player2Tid = playerQueue.pop();

    RPSMessage msg;

    // Find the first available lobby.
    auto lobbyIt = kit::find_if(lobbies, &lobbies[MAX_LOBBY], [](const Lobby& lobby) { return !lobby.isPlaying; });

    if (lobbyIt == &lobbies[MAX_LOBBY]) {
      // No lobby available.
      msg = {.type = RPSMessageType::NO_AVAILABLE_LOBBY, .noAvailableLobbyMessage = NoAvailableLobbyMessage{}};

    } else {
      // Found a lobby.
      *lobbyIt = Lobby{true, player1Tid, player2Tid, PlayType::EMPTY, PlayType::EMPTY};
      int lobbyId = int(lobbyIt - lobbies);
      msg = {.type = RPSMessageType::LOBBY_ASSIGNED, .lobbyAssignedMessage = LobbyAssignedMessage{lobbyId}};
    }
    doReply(player1Tid, msg);
    doReply(player2Tid, msg);
  };

  const auto playHandler = [&](int senderTid, const PlayMessage& request) {
    Lobby& lobby = lobbies[request.lobbyId];

    if (!lobby.isPlaying) {
      doReply(senderTid, {.type = RPSMessageType::QUIT, .quitMessage = QuitMessage{request.lobbyId}});
      return;
    }

    // Player plays the move.
    if (lobby.player1Tid == senderTid) {
      lobby.player1Play = request.play;
    } else {
      lobby.player2Play = request.play;
    }

    // Both players have played their moves. Reply with the result.
    if (lobby.player1Play != PlayType::EMPTY && lobby.player2Play != PlayType::EMPTY) {
      RPSMessage reply = {.type = RPSMessageType::MATCH_RESULT,
                          .matchResultMessage = MatchResultMessage{lobby.player1Play, lobby.player2Play}};
      doReply(lobby.player1Tid, reply);
      reply = {.type = RPSMessageType::MATCH_RESULT,
               .matchResultMessage = MatchResultMessage{lobby.player2Play, lobby.player1Play}};
      doReply(lobby.player2Tid, reply);
      // Reset the lobby but continue playing.
      lobby.player1Play = PlayType::EMPTY;
      lobby.player2Play = PlayType::EMPTY;
    }
  };

  const auto quitHandler = [&](int senderTid, const QuitMessage& request) {
    Lobby& lobby = lobbies[request.lobbyId];
    lobby.isPlaying = false;
    RPSMessage reply = {.type = RPSMessageType::QUIT, .quitMessage = request};
    doReply(senderTid, reply);
    if (lobby.player1Tid == senderTid && lobby.player2Play != PlayType::EMPTY) {
      doReply(lobby.player2Tid, reply);
    } else if (lobby.player2Tid == senderTid && lobby.player1Play != PlayType::EMPTY) {
      doReply(lobby.player1Tid, reply);
    }
  };

  // Handle all the requests.
  for (;;) {
    int senderTid;
    RPSMessage request;
    ::Receive(&senderTid, reinterpret_cast<char*>(&request), sizeof(RPSMessage));

    switch (request.type) {
    case RPSMessageType::SIGNUP:
      signUpHandler(senderTid);
      break;
    case RPSMessageType::PLAY:
      playHandler(senderTid, request.playMessage);
      break;
    case RPSMessageType::QUIT:
      quitHandler(senderTid, request.quitMessage);
      break;
    default:
      logError("invalid and impossible client request");
    }
  }
}

template <bool Interactive> void rpsClientTask() {
  int lobbyId = -1;
  char buffer[128];
  const auto log = [&lobbyId, tid = ::MyTid()](const char* msg, bool newline = true) {
    static char lBuf[128];
    if (lobbyId == -1) {
      kit::formatString(lBuf, "Client[%d] ", tid);
    } else {
      kit::formatString(lBuf, "Client[%d] Lobby[%d] ", tid, lobbyId);
    }
    Uart::syncPrint(lBuf);
    Uart::syncPrint(msg);
    if (newline) {
      Uart::syncPrint("\r\n");
    }
  };

  int serverTid = WhoIs(RPS_SERVER_NAME);
  if (serverTid < 0) {
    log("Failed to find RPS server!");
    return;
  }

  RPSMessage reply;
  const auto send = [&](const RPSMessage& request) {
    if (::Send(serverTid, reinterpret_cast<const char*>(&request), sizeof(RPSMessage), reinterpret_cast<char*>(&reply),
               sizeof(RPSMessage)) < 0) {
      log("Failed to send request to server!");
      return false;
    }
    return true;
  };

  // Enroll into the challenger queue.
  if (!send({.type = RPSMessageType::SIGNUP, .signUpMessage = SignUpMessage{}})) {
    log("Failed to send signup request to server!");
    return;
  }

  // No available lobby.
  if (reply.type == RPSMessageType::NO_AVAILABLE_LOBBY) {
    log("No available lobby!");
    return;
  }
  lobbyId = reply.lobbyAssignedMessage.lobbyId;

  PlayType play;
  if constexpr (Interactive) {
    log("Enter your move [r]ock, [p]aper, [s]cissor or any other letter to quit: ", false);

    // Play the move.
    play = [&buffer]() {
      char c = Uart::syncRead(Uart::CONSOLE);
      kit::formatString(buffer, "%c\r\n", c);
      Uart::syncPrint(buffer);

      if (c == 'r' || c == 'R')
        return PlayType::ROCK;
      if (c == 'p' || c == 'P')
        return PlayType::PAPER;
      if (c == 's' || c == 'S')
        return PlayType::SCISSORS;
      return PlayType::EMPTY;
    }();

    // Player wants to quit.
    if (play == PlayType::EMPTY) {
      if (!send({.type = RPSMessageType::QUIT, .quitMessage = QuitMessage{lobbyId}})) {
        log("Failed to send quit request to server!");
        return;
      }
      log("You quit the game. The game is cancelled!");
      return;
    }
  } else {
    // Non-interactive mode, "random" play.
    play = static_cast<PlayType>((timer::system_timer.now() % 3) + 1);
  }

  // Player wants to play.
  if (!send({.type = RPSMessageType::PLAY, .playMessage = PlayMessage{lobbyId, play}})) {
    log("Failed to send play request to server!");
    return;
  }

  // Opponent rage quitted.
  if (reply.type == RPSMessageType::QUIT) {
    log("Opponent quit the game. The game is cancelled!");
    return;
  }

  // Result;
  const char* result = [](PlayType me, PlayType you) {
    if (me == you) {
      return "draw";
    }
    switch (me) {
    case PlayType::ROCK:
      return (you == PlayType::SCISSORS) ? "win" : "lose";
    case PlayType::PAPER:
      return (you == PlayType::ROCK) ? "win" : "lose";
    case PlayType::SCISSORS:
      return (you == PlayType::PAPER) ? "win" : "lose";
    default:
      logError("impossible result");
    }
  }(reply.matchResultMessage.myPlay, reply.matchResultMessage.opponentPlay);

  kit::formatString(buffer, "You played %s, opponent played %s, result %s.",
                    playToString(reply.matchResultMessage.myPlay), playToString(reply.matchResultMessage.opponentPlay),
                    result);
  log(buffer);

  // Quit the game.
  if (!send({.type = RPSMessageType::QUIT, .quitMessage = QuitMessage{lobbyId}})) {
    log("Failed to send quit request to server!");
    return;
  }
  if (reply.type != RPSMessageType::QUIT) {
    log("Unexpected response from server when quitting the game!");
    return;
  }

  log("Game finished normally!");
}

void TestInteractive() {
  ::Create(Priority::MEDIUM, rpsClientTask<true>);
  ::Create(Priority::MEDIUM, rpsClientTask<true>);
}

template <Priority P> void TestNonInteractive() {
  for (int i = 0; i < 32; ++i) {
    ::Create(P, rpsClientTask<false>);
    ::Create(P, rpsClientTask<false>);
  }
}

} // namespace

namespace k2 {

void FirstUserTask() {
  // Creates the name server.
  name_server::createNameServerTask(Priority::MEDIUM);

  // Creates the Rock/Paper/Scissors server.
  ::Create(Priority::HIGH, rpsServerTask);

  // Creates the Rock/Paper/Scissors clients.

  Uart::syncPrint("Test 1: Interactive clients... press any key to start...");
  Uart::syncRead(Uart::CONSOLE);
  Uart::syncPrint("\r\n");
  ::Create(Priority::MEDIUM, TestInteractive);

  Uart::syncPrint(
      Uart::CONSOLE,
      "Test 2: Medium (lower than server) priority clients with automatic random play... press any key to start...");
  Uart::syncRead(Uart::CONSOLE);
  Uart::syncPrint("\r\n");
  ::Create(Priority::MEDIUM, TestNonInteractive<Priority::MEDIUM>);

  Uart::syncPrint(
      Uart::CONSOLE,
      "Test 3: High (same as server) priority clients with automatic random play... press any key to start...");
  Uart::syncRead(Uart::CONSOLE);
  Uart::syncPrint("\r\n");
  ::Create(Priority::MEDIUM, TestNonInteractive<Priority::HIGH>);
}
} // namespace k2
