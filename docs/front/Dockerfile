FROM node:latest as builder
COPY package.json package-lock.json ./
RUN npm install && mkdir /front && mv ./node_modules ./front
WORKDIR /front
COPY . .
RUN npm run ng build --


FROM nginx:alpine
#!/bin/sh
COPY ./nginx/nginx.conf /etc/nginx/nginx.conf
RUN rm -rf /workspace/nginx/html/*
COPY --from=builder /front/dist/docs /workspace/nginx/html
EXPOSE 4200 80
ENTRYPOINT ["nginx", "-g", "daemon off;"]
