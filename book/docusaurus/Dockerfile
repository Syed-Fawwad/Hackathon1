FROM node:18-alpine

WORKDIR /app

COPY book/docusaurus/package*.json ./

RUN npm install

COPY book/docusaurus/ ./

RUN npm run build

EXPOSE 3000

CMD ["npm", "run", "serve"]