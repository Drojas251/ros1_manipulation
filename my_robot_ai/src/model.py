import pandas as pd
import random
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from tensorflow.keras.preprocessing.text import Tokenizer
from tensorflow.keras.preprocessing.sequence import pad_sequences
import tensorflow as tf
from tensorflow.keras.layers import Input, Embedding, GlobalAveragePooling1D, Dense, concatenate
from colorama import Fore
from data_set import build_data_set


class ActionChooser:
    def __init__(self):
        data = self._gen_data()
        df = pd.DataFrame(data)
        self.train(df)

    def train(self, df):
        # Tokenize input
        self.tokenizer = Tokenizer()
        self.tokenizer.fit_on_texts(df['text'])
        self.X = self.tokenizer.texts_to_sequences(df['text'])
        self.X = pad_sequences(self.X)

        # Label encoding
        self.label_encoder_size = LabelEncoder()
        self.label_encoder_color = LabelEncoder()

        y_size = self.label_encoder_size.fit_transform(df['size'])
        y_color = self.label_encoder_color.fit_transform(df['color'])

        # Train-test split
        X_train, X_test, y_size_train, y_size_test, y_color_train, y_color_test = train_test_split(
            self.X, y_size, y_color, test_size=0.2, random_state=42
        )

        # Model
        embedding_layer = Embedding(input_dim=len(self.tokenizer.word_index) + 1, output_dim=64)
        text_input = Input(shape=(self.X.shape[1],), name='text_input')
        embedded_text = embedding_layer(text_input)
        pooled_text = GlobalAveragePooling1D()(embedded_text)
        dense_layer = Dense(32, activation='relu')(pooled_text)

        size_output = Dense(len(self.label_encoder_size.classes_), activation='softmax', name='size_output')(dense_layer)
        color_output = Dense(len(self.label_encoder_color.classes_), activation='softmax', name='color_output')(dense_layer)

        self.model = tf.keras.Model(inputs=text_input, outputs=[size_output, color_output])

        self.model.compile(optimizer='adam',
                    loss={'size_output': 'sparse_categorical_crossentropy', 'color_output': 'sparse_categorical_crossentropy'},
                    metrics={'size_output': 'accuracy', 'color_output': 'accuracy'})

        # Model training
        self.model.fit(X_train, {'size_output': y_size_train, 'color_output': y_color_train}, epochs=10)

        # Model evaluation
        eval_results = self.model.evaluate(X_test, {'size_output': y_size_test, 'color_output': y_color_test})
        print("Model Evaluation:")
        print(f"Size Accuracy: {eval_results[3] * 100:.2f}%")
        print(f"Color Accuracy: {eval_results[4] * 100:.2f}%")

    def predict(self, text):

        # Tokenize and pad the new text
        new_text_sequences = self.tokenizer.texts_to_sequences(text)
        new_text_padded = pad_sequences(new_text_sequences, maxlen=self.X.shape[1])

        # Make predictions
        predictions = self.model.predict(new_text_padded)

        # Decode the predictions
        predicted_size = self.label_encoder_size.inverse_transform(tf.argmax(predictions[0], axis=1).numpy())
        predicted_color = self.label_encoder_color.inverse_transform(tf.argmax(predictions[1], axis=1).numpy())

        # Print the predictions
        print(Fore.YELLOW + f"User input: {text}" + Fore.WHITE)
        print(f"Predicted Block: {predicted_size[0]} block")
        print(f"Predicted Location: {predicted_color[0]} platform")

        return predicted_size[0], predicted_color[0]

    def _gen_data(self):
        return build_data_set()


