const path = require('path');
const HtmlWebpackPlugin = require('html-webpack-plugin');
const CopyPlugin = require('copy-webpack-plugin');

module.exports = {
	mode: 'development',
	entry: {
		index: './src/index.js',
	},
	devServer: {
		static: {
			directory: path.join(__dirname, 'dist'),
		},
		host: '0.0.0.0',
		server: 'https',
		compress: true,
		port: 8081,
		client: {
			overlay: { warnings: false, errors: true },
		},
		proxy: [{
			context: ['/api'],
			target: 'ws://localhost:8765',
			secure: false,
			ws: true,
			logLevel: 'debug'
		}],
	},
	output: {
		filename: '[name].bundle.js',
		path: path.resolve(__dirname, 'dist'),
		clean: true,
	},
	plugins: [
		new HtmlWebpackPlugin({
			template: './src/index.html',
		}),
		new CopyPlugin({
			patterns: [{ from: 'src/assets', to: 'assets' }],
		}),
	],
	devtool: 'source-map',
};
