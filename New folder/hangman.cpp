#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include <map>
using namespace std;

class Hangman {
private:
    string word_to_guess;
    string guessed_word;
    vector<char> incorrect_guesses;
    int max_attempts;
    int attempts_left;
    map<string, string> word_clues = {
        {"apple", "A sweet, round fruit that grows on trees"},
        {"bread", "A staple food made from flour and baked"},
        {"water", "Clear liquid essential for life"},
        {"house", "A building where people live"},
        {"chair", "A piece of furniture with legs and a seat"},
        {"book", "Collection of pages with written or printed information"},
        {"phone", "Device used for communication"},
        {"shirt", "Clothing worn on the upper body"},
        {"shirt", "A garment worn on the upper body"},
        {"table", "Flat surface used for eating or working"},
        {"flower", "Colorful part of a plant often used for decoration"},
        {"tree", "Tall plant with a trunk and branches"},
        {"car", "Vehicle used for transportation"},
        {"dog", "A common pet and domestic animal"},
        {"cat", "A furry household pet"},
        {"sun", "The bright object that provides light during the day"},
        {"moon", "The natural satellite that appears at night"},
        {"cake", "A sweet dessert typically eaten at celebrations"},
        {"shoe", "Footwear worn to protect feet"},
        {"ball", "A round object used in many sports"},
        {"door", "An entrance or exit to a room or building"},
        {"window", "A glass opening in a wall for light and air"},
        {"plate", "A flat dish used for serving food"},
        {"fork", "Eating utensil with prongs"},
        {"spoon", "Eating utensil with a curved part"},
        {"glass", "Container used for drinking"},
        {"coffee", "A popular hot beverage"},
        {"milk", "A white liquid produced by cows"},
        {"bread", "A staple food made from flour"},
        {"butter", "A dairy product spread on bread"},
        {"cheese", "A dairy product made from milk"},
        {"juice", "Liquid extracted from fruits"},
        {"paper", "Material used for writing"},
        {"pencil", "Writing tool with graphite"},
        {"music", "Arrangement of sounds in a pleasing way"},
        {"dance", "Rhythmic body movement"},
        {"light", "Something that makes things visible"},
        {"heart", "Organ that pumps blood in the body"},
        {"smile", "Expression of happiness"},
        {"cloud", "Visible mass of water droplets in the sky"},
        {"river", "Large flowing body of water"},
        {"mountain", "Very high natural landform"},
        {"ocean", "Large body of salt water"},
        {"friend", "Person who is close and supportive"},
        {"family", "Group of people related by blood or marriage"},
        {"school", "Place of learning"},
        {"sleep", "Natural state of rest"},
        {"dream", "Series of thoughts during sleep"},
        {"love", "Strong feeling of affection"}
    };

public:
    Hangman() {
        max_attempts = 6;              // Change the max number of guesses
        attempts_left = max_attempts;
    }

    void setWordToGuess() {
        vector<string> words;
        for (auto& pair : word_clues) {
            words.push_back(pair.first);
        }
        
        srand(time(0));
        word_to_guess = words[rand() % words.size()];
        guessed_word = string(word_to_guess.length(), '_');
        
        // Reset game state
        incorrect_guesses.clear();
        attempts_left = max_attempts;
    }

    void displayWord() {
        cout << "Word: " << guessed_word << endl;
    }

    void displayIncorrectGuesses() {
        cout << "Incorrect guesses: ";
        for (char guess : incorrect_guesses) {
            cout << guess << " ";
        }
        cout << endl;
    }

    void showClue() {
        if (word_clues.find(word_to_guess) != word_clues.end()) {
            cout << "Clue: " << word_clues[word_to_guess] << endl;
        }
    }

    void showLetterCount() {
        cout << "Number of letters in the word: " << word_to_guess.length() << endl;
    }

    bool guessLetter(char letter) {
        bool found = false;
        for (int i = 0; i < word_to_guess.length(); ++i) {
            if (word_to_guess[i] == letter) {
                guessed_word[i] = letter;
                found = true;
            }
        }
        return found;
    }

    bool hasWon() {
        return word_to_guess == guessed_word;
    }

    bool hasLost() {
        return attempts_left <= 0;
    }

    bool play() {
        setWordToGuess();
        cout << "Welcome to Hangman!" << endl;
        cout << "You have " << max_attempts << " attempts to guess the word." << endl;
        
        char help_choice;
        cout << "Do you want a clue? (y/n): ";
        cin >> help_choice;
        if (help_choice == 'y' || help_choice == 'Y') {
            showClue();
        }

        cout << "Do you want to know the number of letters? (y/n): ";
        cin >> help_choice;
        if (help_choice == 'y' || help_choice == 'Y') {
            showLetterCount();
        }

        while (!hasWon() && !hasLost()) {
            displayWord();
            displayIncorrectGuesses();
            cout << "Attempts left: " << attempts_left << endl;
            cout << "Enter a letter to guess: ";
            char guess;
            cin >> guess;

            if (!isalpha(guess)) {
                cout << "Invalid input. Please enter a letter." << endl;
                continue;
            }

            guess = tolower(guess);

            bool alreadyGuessed = false;
            if (find(incorrect_guesses.begin(), incorrect_guesses.end(), guess) != incorrect_guesses.end()) {
                alreadyGuessed = true;
            }

            if (guessed_word.find(guess) != string::npos) {
                alreadyGuessed = true;
            }

            if (alreadyGuessed) {
                cout << "You already guessed that letter!" << endl;
                continue;
            }

            if (!guessLetter(guess)) {
                incorrect_guesses.push_back(guess);
                --attempts_left;
            }
            cout << endl;
        }

        if (hasWon()) {
            cout << "Congratulations! You guessed the word: " << word_to_guess << endl;
            return true;
        } else if (hasLost()) {
            cout << "Game Over! The word was: " << word_to_guess << endl;
            return false;
        }
        return false;
    }
};

int main() {
    Hangman game;
    char play_again;

    do {
        game.play();

        cout << "Do you want to play again? (y/n): ";
        cin >> play_again;
    } while (play_again == 'y' || play_again == 'Y');

    cout << "Thanks for playing Hangman!" << endl;
    return 0;
}
